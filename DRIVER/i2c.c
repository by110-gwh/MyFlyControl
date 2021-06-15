#include "i2c.h"
#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "queue.h"

#define I2C_EVENT_SUCCESSFUL 1 << 0
#define I2C_EVENT_ERROR 1 << 1

//I2C消息标志队列
static QueueHandle_t i2c_queue;
//I2C外设句柄
I2C_HandleTypeDef hi2c1;


/**********************************************************************************************************
*函 数 名: i2c_fail_recover
*功能说明: I2C错误恢复
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void i2c_fail_recover(void)
{
    int j;
    int nRetry = 0;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0 || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 0) {
        //配置I2C引脚为开漏输出
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        do{
            //产生CLK脉冲
            j = 50;
            while (--j);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            j = 50;
            while (--j);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        //判断SDA是否恢复
        }while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 0 && nRetry++ < 70);
        
        //拉高CLK
        j = 50;
        while (--j);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        //SDA产生一个脉冲
        j = 50;
        while (--j);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        j = 50;
        while (--j);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        
        //恢复GPIO配置
        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    //重新初始化I2C
	HAL_I2C_Init(&hi2c1);
}

/**********************************************************************************************************
*函 数 名: Spi_GPIO_Init
*功能说明: SPI从机设备CS引脚初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void i2c_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	//外设时钟使能
	__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    
	//IO初始化
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
	//I2C初始化
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&hi2c1);
    
    i2c_fail_recover();
	
	//I2C中断初始化
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
	
	i2c_queue = xQueueCreate(1, sizeof(uint8_t));
}

/**********************************************************************************************************
*函 数 名: i2c_transmit
*功能说明: i2c传输
*形    参: 7bit从机地址 是否为写 写入数据 写入数量
*返 回 值: 写入状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c_transmit(uint8_t slave_address, uint8_t is_write, uint8_t *pdata, uint8_t count)
{
    uint8_t res;
    
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
        i2c_fail_recover();
    
    if (is_write) {
        HAL_I2C_Master_Transmit_IT(&hi2c1, slave_address << 1, pdata, count);
    } else {
        HAL_I2C_Master_Receive_IT(&hi2c1, slave_address << 1, pdata, count);
    }
    
    xQueueReceive(i2c_queue, (void *) &res, portMAX_DELAY);
	if (res & I2C_EVENT_ERROR)
		return -1;
	return 0;
}

/**********************************************************************************************************
*函 数 名: i2c_single_write
*功能说明: 查询方式进行单个寄存器写入
*形    参: 7bit从机地址 寄存器地址 写入数据
*返 回 值: 写入状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c_single_write(uint8_t slave_address, uint8_t reg_address, uint8_t reg_data)
{
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
        i2c_fail_recover();
    
	if (HAL_I2C_Mem_Write(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 1) != HAL_OK)
		return -1;
	return 0;
}

/**********************************************************************************************************
*函 数 名: i2c_single_write_it
*功能说明: 中断方式进行单个寄存器写入
*形    参: 7bit从机地址 寄存器地址 写入数据
*返 回 值: 写入状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c_single_write_it(uint8_t slave_address, uint8_t reg_address, uint8_t reg_data)
{
	uint8_t res;
    
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
        i2c_fail_recover();
    
	if (HAL_I2C_Mem_Write_IT(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, &reg_data, 1) != HAL_OK)
		return -1;
	
	xQueueReceive(i2c_queue, (void *) &res, portMAX_DELAY);
	if (res & I2C_EVENT_ERROR)
		return -1;
	return 0;
}

/**********************************************************************************************************
*函 数 名: i2c_single_read
*功能说明: 查询方式进行单个寄存器读取
*形    参: 7bit从机地址 寄存器地址 读取数据
*返 回 值: 读取状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c_single_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data)
{
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
        i2c_fail_recover();
    
	if (HAL_I2C_Mem_Read(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, reg_data, 1, 1) != HAL_OK)
		return -1;
	return 0;
}

/**********************************************************************************************************
*函 数 名: i2c_single_read_it
*功能说明: 中断方式进行单个寄存器读取
*形    参: 7bit从机地址 寄存器地址 读取数据
*返 回 值: 读取状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c_single_read_it(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data)
{
	uint8_t res;
    
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
        i2c_fail_recover();
    
	if (HAL_I2C_Mem_Read_IT(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, reg_data, 1) != HAL_OK)
		return -1;
	
	xQueueReceive(i2c_queue, (void *) &res, portMAX_DELAY);
	if (res & I2C_EVENT_ERROR)
		return -1;
	return 0;
}

/**********************************************************************************************************
*函 数 名: i2c_multi_read
*功能说明: 查询方式进行多个寄存器读取
*形    参: 7bit从机地址 寄存器地址 读取数据 读取个数
*返 回 值: 读取状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c_multi_read(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt)
{
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
        i2c_fail_recover();
    
	if (HAL_I2C_Mem_Read(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, reg_data, read_cnt, 1) != HAL_OK)
		return -1;
	return 0;
}

/**********************************************************************************************************
*函 数 名: i2c_multi_read_it
*功能说明: 中断方式进行多个寄存器读取
*形    参: 7bit从机地址 寄存器地址 读取数据 读取个数
*返 回 值: 读取状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c_multi_read_it(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt)
{
	uint8_t res;
    
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
        i2c_fail_recover();
    
	if (HAL_I2C_Mem_Read_IT(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, reg_data, read_cnt) != HAL_OK)
		return -1;
	
	xQueueReceive(i2c_queue, (void *) &res, portMAX_DELAY);
	if (res & I2C_EVENT_ERROR)
		return -1;
	
	return 0;
}

/**********************************************************************************************************
*函 数 名: i2c_multi_write
*功能说明: 查询方式进行多个寄存器写入
*形    参: 7bit从机地址 寄存器地址 读取数据 读取个数
*返 回 值: 读取状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c_multi_write(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt)
{
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
        i2c_fail_recover();
    
	if (HAL_I2C_Mem_Write(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, reg_data, read_cnt, 1) != HAL_OK)
		return -1;
	return 0;
}

/**********************************************************************************************************
*函 数 名: i2c_multi_write_it
*功能说明: 中断方式进行多个寄存器写入
*形    参: 7bit从机地址 寄存器地址 读取数据 读取个数
*返 回 值: 读取状态：0：成功，-1：错误
**********************************************************************************************************/
int i2c_multi_write_it(uint8_t slave_address, uint8_t reg_address, uint8_t *reg_data, uint8_t read_cnt)
{
	uint8_t res;
    
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
        i2c_fail_recover();
    
	if (HAL_I2C_Mem_Write_IT(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, reg_data, read_cnt) != HAL_OK)
		return -1;
	
	xQueueReceive(i2c_queue, (void *) &res, portMAX_DELAY);
	if (res & I2C_EVENT_ERROR)
		return -1;
	
	return 0;
}

/**********************************************************************************************************
*函 数 名: HAL_I2C_MemTxCpltCallback
*功能说明: HAL的I2C回调函数
*形    参: I2C句柄
*返 回 值: 无
**********************************************************************************************************/
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t data;
	BaseType_t xResult;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	data = I2C_EVENT_SUCCESSFUL;
	xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
	if(xResult == pdPASS)
		  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**********************************************************************************************************
*函 数 名: HAL_I2C_MemRxCpltCallback
*功能说明: HAL的I2C回调函数
*形    参: I2C句柄
*返 回 值: 无
**********************************************************************************************************/
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t data;
	BaseType_t xResult;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	data = I2C_EVENT_SUCCESSFUL;
	xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
	if(xResult == pdPASS)
		  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**********************************************************************************************************
*函 数 名: HAL_I2C_ErrorCallback
*功能说明: HAL的I2C回调函数
*形    参: I2C句柄
*返 回 值: 无
**********************************************************************************************************/
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t data;
	BaseType_t xResult;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	data = I2C_EVENT_ERROR;
	xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
	if(xResult == pdPASS)
		  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    
    i2c_fail_recover();
}

/**********************************************************************************************************
*函 数 名: HAL_I2C_MasterTxCpltCallback
*功能说明: HAL的I2C回调函数
*形    参: I2C句柄
*返 回 值: 无
**********************************************************************************************************/
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t data;
	BaseType_t xResult;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	data = I2C_EVENT_SUCCESSFUL;
	xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
	if(xResult == pdPASS)
		  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**********************************************************************************************************
*函 数 名: HAL_I2C_MasterRxCpltCallback
*功能说明: HAL的I2C回调函数
*形    参: I2C句柄
*返 回 值: 无
**********************************************************************************************************/
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t data;
	BaseType_t xResult;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	data = I2C_EVENT_SUCCESSFUL;
	xResult = xQueueSendFromISR(i2c_queue, &data, &xHigherPriorityTaskWoken);
	if(xResult == pdPASS)
		  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
