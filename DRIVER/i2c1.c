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

//I2C��Ϣ��־����
static QueueHandle_t i2c_queue;
//���ݻ�����
static uint8_t *g_pdata;
//д����
static uint8_t g_is_write;
//ʣ���ֽ�
static uint32_t last_len;

void I2C1_IRQHandle(void);

/**********************************************************************************************************
*�� �� ��: i2c_fail_recover
*����˵��: I2C����ָ�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void i2c_fail_recover(void)
{
    if ((I2CMasterLineStateGet(I2C1_BASE) & 0x3) != 0x3) {
        //����SCK����Ϊ���
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_6 | GPIO_PIN_7);
        GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
        do{
            //����CLK����
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
            ROM_SysCtlDelay(50);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
            ROM_SysCtlDelay(50);
        //�ж�SDA�Ƿ�ָ�
        }while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) == 0);
        
        //����SDA����Ϊ���
        ROM_SysCtlDelay(50);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
        GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
        //����ֹͣ�ź�
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
        ROM_SysCtlDelay(50);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
        ROM_SysCtlDelay(50);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
        
        //�ָ�GPIO����
        GPIOPinConfigure(GPIO_PA6_I2C1SCL);
        GPIOPinConfigure(GPIO_PA7_I2C1SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
        GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    }
}

/**********************************************************************************************************
*�� �� ��: i2c1_init
*����˵��: I2C1��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void i2c1_init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //��ʼ��GPIO
    ROM_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    ROM_GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    
    vTaskDelay(100);
    
    //��ʼ��I2C����
    ROM_I2CMasterInitExpClk(I2C1_BASE, ROM_SysCtlClockGet(), true);
    
    //���߸�λ
    i2c_fail_recover();
    
    //ʹ��I2C�ж�
    ROM_I2CMasterIntEnable(I2C1_BASE);
    ROM_IntPrioritySet(INT_I2C1, 3 << 5);
    IntRegister(INT_I2C1, I2C1_IRQHandle);
    ROM_IntEnable(INT_I2C1);
    
    //��ʼ����Ϣ����
	i2c_queue = xQueueCreate(1, sizeof(uint8_t));
}

/**********************************************************************************************************
*�� �� ��: I2C1_IRQHandle
*����˵��: I2C1�жϺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void I2C1_IRQHandle(void)
{
    //I2Cд����
    if (g_is_write) {
        //I2C����
        if (I2CMasterErr(I2C1_BASE)) {
            uint8_t data;
            BaseType_t xResult;
            last_len = 0;
            //����ֹͣλ
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            //��֪ͨ����
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            data = I2C_EVENT_ERROR;
            xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
            if(xResult == pdPASS)
                  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        //I2C�������
        } else if (last_len == 0) {
            //��֪ͨ�ɹ�
            uint8_t data;
            BaseType_t xResult;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            data = I2C_EVENT_SUCCESSFUL;
            xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
            if(xResult == pdPASS)
                  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        //���һ���ֽ�
        } else if (last_len == 1) {
            //��������
            I2CMasterDataPut(I2C1_BASE, *++g_pdata);
            //����ֹͣ�ź�
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
            last_len = 0;
        } else {
            //��������
            I2CMasterDataPut(I2C1_BASE, *++g_pdata);
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
            last_len--;
        }
    //I2C������
    } else {
        //I2C����
        if (I2CMasterErr(I2C1_BASE)) {
            uint8_t data;
            BaseType_t xResult;
            last_len = 0;
            //����ֹͣλ
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            //��֪ͨ����
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            data = I2C_EVENT_ERROR;
            xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
            if(xResult == pdPASS)
                  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        //I2C�������
        } else if (last_len == 0) {
            //��֪ͨ�ɹ�
            uint8_t data;
            BaseType_t xResult;
            *g_pdata++ = I2CMasterDataGet(I2C1_BASE);
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            data = I2C_EVENT_SUCCESSFUL;
            xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
            if(xResult == pdPASS)
                  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        //���һ���ֽ�
        } else if (last_len == 1) {
            //��������
            *g_pdata++ = I2CMasterDataGet(I2C1_BASE);
            //����ֹͣ�ź�
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            last_len = 0;
        } else {
            //��������
            *g_pdata++ = I2CMasterDataGet(I2C1_BASE);
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            last_len--;
        }
    }
    //���I2C�ж�
    I2CMasterIntClear(I2C1_BASE);
}

/**********************************************************************************************************
*�� �� ��: i2c1_transmit
*����˵��: i2c1����
*��    ��: 7bit�ӻ���ַ �Ƿ�Ϊд д������ д������
*�� �� ֵ: д��״̬��0���ɹ���-1������
**********************************************************************************************************/
int i2c1_transmit(uint8_t slave_address, uint8_t is_write, uint8_t *pdata, uint8_t count)
{
    uint8_t res;
    //�����������ڴ���
    if (last_len)
        return -1;
    
    //I2Cд����
    if (is_write) {
        //����������ַ
        I2CMasterSlaveAddrSet(I2C1_BASE, slave_address, false);
        //���δ���
        if (count == 1) {
            g_pdata = pdata;
            last_len = 0;
            g_is_write = 1;
            //��������
            I2CMasterDataPut(I2C1_BASE, *pdata);
            //��������
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
        } else {
            g_pdata = pdata;
            g_is_write = 1;
            last_len = count - 1;
            //��������
            I2CMasterDataPut(I2C1_BASE, *pdata);
            //��������
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        }
    //I2C������
    } else {
        //����������ַ
        I2CMasterSlaveAddrSet(I2C1_BASE, slave_address, true);
        //���δ���
        if (count == 1) {
            g_pdata = pdata;
            g_is_write = 0;
            last_len = 0;
            //��������
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        } else {
            g_pdata = pdata;
            g_is_write = 0;
            last_len = count - 1;
            //��������
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        }
        
    }
    
    //�ȴ�I2C�������
    xQueueReceive(i2c_queue, (void *) &res, portMAX_DELAY);
	if (res & I2C_EVENT_ERROR) {
        i2c_fail_recover();
		return -1;
    }
	
	return 0;
}

/**********************************************************************************************************
*�� �� ��: i2c1_single_write
*����˵��: ��ѯ��ʽ���е����Ĵ���д��
*��    ��: 7bit�ӻ���ַ �Ĵ�����ַ д������
*�� �� ֵ: д��״̬��0���ɹ���-1������
**********************************************************************************************************/
int i2c1_single_write(uint8_t slave_address, uint8_t reg_address, uint8_t reg_data)
{
    uint8_t buffer[2];
    
    buffer[0] = reg_address;
    buffer[1] = reg_data;
    return i2c1_transmit(slave_address, 1, buffer, 2);
}

/**********************************************************************************************************
*�� �� ��: i2c1_single_read
*����˵��: �����Ĵ�����ȡ
*��    ��: 7bit�ӻ���ַ �Ĵ�����ַ ��ȡ����
*�� �� ֵ: ��ȡ״̬��0���ɹ���-1������
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
*�� �� ��: i2c1_multi_read
*����˵��: ����Ĵ�����ȡ
*��    ��: 7bit�ӻ���ַ �Ĵ�����ַ ��ȡ���� ��ȡ����
*�� �� ֵ: ��ȡ״̬��0���ɹ���-1������
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
*�� �� ��: i2c1_multi_write
*����˵��: ����Ĵ���д��
*��    ��: 7bit�ӻ���ַ �Ĵ�����ַ ��ȡ���� ��ȡ����
*�� �� ֵ: ��ȡ״̬��0���ɹ���-1������
**********************************************************************************************************/
int i2c1_multi_write(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt)
{
    uint8_t res;
    //�����������ڴ���
    if (last_len)
        return -1;
    
    //����������ַ
    I2CMasterSlaveAddrSet(I2C1_BASE, slave_address, false);
    g_pdata = reg_data - 1;
    g_is_write = 1;
    last_len = read_cnt - 1;
    //��������
    I2CMasterDataPut(I2C1_BASE, reg_address);
    //��������
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    
    //�ȴ�I2C�������
    xQueueReceive(i2c_queue, (void *) &res, portMAX_DELAY);
	if (res & I2C_EVENT_ERROR) {
        i2c_fail_recover();
		return -1;
    }
	
	return 0;
}
