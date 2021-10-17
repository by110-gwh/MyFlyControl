#include "i2c1.h"

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define I2C_EVENT_SUCCESSFUL 1 << 0
#define I2C_EVENT_ERROR 1 << 1

//I2C消息标志队列
static QueueHandle_t i2c_queue;
//数据缓冲区
static uint8_t *g_pdata;
//写操作
static uint8_t g_is_write;
//剩余字节
static uint32_t last_len;

void I2C1_IRQHandle(void);

/**********************************************************************************************************
*函 数 名: i2c_fail_recover
*功能说明: I2C错误恢复
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void i2c_fail_recover(void)
{
    if ((I2CMasterLineStateGet(I2C1_BASE) & 0x3) != 0x3) {
        //配置SCK引脚为输出
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_6 | GPIO_PIN_7);
        GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
        do{
            //产生CLK脉冲
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
            ROM_SysCtlDelay(50);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
            ROM_SysCtlDelay(50);
        //判断SDA是否恢复
        }while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) == 0);
        
        //配置SDA引脚为输出
        ROM_SysCtlDelay(50);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
        GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
        //产生停止信号
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
        ROM_SysCtlDelay(50);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
        ROM_SysCtlDelay(50);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
        
        //恢复GPIO配置
        GPIOPinConfigure(GPIO_PA6_I2C1SCL);
        GPIOPinConfigure(GPIO_PA7_I2C1SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
        GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    }
}

/**********************************************************************************************************
*函 数 名: i2c1_init
*功能说明: I2C1初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void i2c1_init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //初始化GPIO
    ROM_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    ROM_GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    
    vTaskDelay(100);
    
    //初始化I2C外设
    ROM_I2CMasterInitExpClk(I2C1_BASE, ROM_SysCtlClockGet(), true);
    
    //总线复位
    i2c_fail_recover();
    
    //使能I2C中断
    ROM_I2CMasterIntEnable(I2C1_BASE);
    ROM_IntPrioritySet(INT_I2C1, 3 << 5);
    IntRegister(INT_I2C1, I2C1_IRQHandle);
    ROM_IntEnable(INT_I2C1);
    
    //初始化消息队列
	i2c_queue = xQueueCreate(1, sizeof(uint8_t));
}

/**********************************************************************************************************
*函 数 名: I2C1_IRQHandle
*功能说明: I2C1中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void I2C1_IRQHandle(void)
{
    //I2C写操作
    if (g_is_write) {
        //I2C出错
        if (I2CMasterErr(I2C1_BASE)) {
            uint8_t data;
            BaseType_t xResult;
            last_len = 0;
            //发出停止位
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            //发通知错误
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            data = I2C_EVENT_ERROR;
            xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
            if(xResult == pdPASS)
                  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        //I2C传输完成
        } else if (last_len == 0) {
            //发通知成功
            uint8_t data;
            BaseType_t xResult;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            data = I2C_EVENT_SUCCESSFUL;
            xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
            if(xResult == pdPASS)
                  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        //最后一个字节
        } else if (last_len == 1) {
            //发送数据
            I2CMasterDataPut(I2C1_BASE, *++g_pdata);
            //发出停止信号
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
            last_len = 0;
        } else {
            //发送数据
            I2CMasterDataPut(I2C1_BASE, *++g_pdata);
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
            last_len--;
        }
    //I2C读操作
    } else {
        //I2C出错
        if (I2CMasterErr(I2C1_BASE)) {
            uint8_t data;
            BaseType_t xResult;
            last_len = 0;
            //发出停止位
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            //发通知错误
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            data = I2C_EVENT_ERROR;
            xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
            if(xResult == pdPASS)
                  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        //I2C传输完成
        } else if (last_len == 0) {
            //发通知成功
            uint8_t data;
            BaseType_t xResult;
            *g_pdata++ = I2CMasterDataGet(I2C1_BASE);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            data = I2C_EVENT_SUCCESSFUL;
            xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
            if(xResult == pdPASS)
                  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        //最后一个字节
        } else if (last_len == 1) {
            //发送数据
            *g_pdata++ = I2CMasterDataGet(I2C1_BASE);
            //发出停止信号
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            last_len = 0;
        } else {
            //发送数据
            *g_pdata++ = I2CMasterDataGet(I2C1_BASE);
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            last_len--;
        }
    }
    //清除I2C中断
    I2CMasterIntClear(I2C1_BASE);
}

/**********************************************************************************************************
*函 数 名: i2c1_transmit
*功能说明: i2c1传输
*形    参: 7bit从机地址 是否为写 写入数据 写入数量
*返 回 值: 写入状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c1_transmit(uint8_t slave_address, uint8_t is_write, uint8_t *pdata, uint8_t count)
{
    uint8_t res;
    //还有数据正在传输
    if (last_len)
        return -1;
    
    //I2C写操作
    if (is_write) {
        //发送器件地址
        I2CMasterSlaveAddrSet(I2C1_BASE, slave_address, false);
        //单次传输
        if (count == 1) {
            g_pdata = pdata;
            last_len = 0;
            g_is_write = 1;
            //发送数据
            I2CMasterDataPut(I2C1_BASE, *pdata);
            //启动传输
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
        } else {
            g_pdata = pdata;
            g_is_write = 1;
            last_len = count - 1;
            //发送数据
            I2CMasterDataPut(I2C1_BASE, *pdata);
            //启动传输
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        }
    //I2C读操作
    } else {
        //发送器件地址
        I2CMasterSlaveAddrSet(I2C1_BASE, slave_address, true);
        //单次传输
        if (count == 1) {
            g_pdata = pdata;
            g_is_write = 0;
            last_len = 0;
            //启动传输
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        } else {
            g_pdata = pdata;
            g_is_write = 0;
            last_len = count - 1;
            //启动传输
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        }
        
    }
    
    //等待I2C传输完成
    xQueueReceive(i2c_queue, (void *) &res, portMAX_DELAY);
	if (res & I2C_EVENT_ERROR) {
        i2c_fail_recover();
		return -1;
    }
	
	return 0;
}

/**********************************************************************************************************
*函 数 名: i2c1_single_write
*功能说明: 查询方式进行单个寄存器写入
*形    参: 7bit从机地址 寄存器地址 写入数据
*返 回 值: 写入状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c1_single_write(uint8_t slave_address, uint8_t reg_address, uint8_t reg_data)
{
    uint8_t buffer[2];
    
    buffer[0] = reg_address;
    buffer[1] = reg_data;
    return i2c1_transmit(slave_address, 1, buffer, 2);
}

/**********************************************************************************************************
*函 数 名: i2c1_single_read
*功能说明: 单个寄存器读取
*形    参: 7bit从机地址 寄存器地址 读取数据
*返 回 值: 读取状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c1_single_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data)
{
    if (i2c1_transmit(slave_address, 1, &reg_address, 1) != 0)
        return -1;
    
    if (i2c1_transmit(slave_address, 0, reg_data, 1) != 0)
        return -1;
    
    return 0;
}

/**********************************************************************************************************
*函 数 名: i2c1_multi_read
*功能说明: 多个寄存器读取
*形    参: 7bit从机地址 寄存器地址 读取数据 读取个数
*返 回 值: 读取状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c1_multi_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt)
{
    if (i2c1_transmit(slave_address, 1, &reg_address, 1) != 0)
        return -1;
    
    if (i2c1_transmit(slave_address, 0, reg_data, read_cnt) != 0)
        return -1;
    
    return 0;
}

/**********************************************************************************************************
*函 数 名: i2c1_multi_write
*功能说明: 多个寄存器写入
*形    参: 7bit从机地址 寄存器地址 读取数据 读取个数
*返 回 值: 读取状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c1_multi_write(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt)
{
    uint8_t res;
    //还有数据正在传输
    if (last_len)
        return -1;
    
    //发送器件地址
    I2CMasterSlaveAddrSet(I2C1_BASE, slave_address, false);
    g_pdata = reg_data - 1;
    g_is_write = 1;
    last_len = read_cnt - 1;
    //发送数据
    I2CMasterDataPut(I2C1_BASE, reg_address);
    //启动传输
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    
    //等待I2C传输完成
    xQueueReceive(i2c_queue, (void *) &res, portMAX_DELAY);
	if (res & I2C_EVENT_ERROR) {
        i2c_fail_recover();
		return -1;
    }
	
	return 0;
}
