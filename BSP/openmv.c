#include "openmv.h"

#include "FreeRTOS.h"
#include "queue.h"

//杆的左右距离
volatile int16_t pole_distance;
//线上下距离
volatile int16_t line_high;
//更新标志
volatile uint32_t openmv_updata_flag;


/**********************************************************************************************************
*函 数 名: openmv_init
*功能说明: openmv初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void openmv_init(void)
{
}

/**********************************************************************************************************
*函 数 名: openmv_rec_callback
*功能说明: openmv接收数据回调函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void openmv_rec_callback(uint8_t data)
{
    static uint8_t openmc_rec_state;
    static uint8_t rec_type;
    static int16_t rec_data_x;
    static int16_t rec_data_y;
    //接收到帧头0xF3
    if (openmc_rec_state == 0 && data == 0xF3) {
        openmc_rec_state = 1;
    //数据类型
    } else if (openmc_rec_state == 1) {
        rec_type = data;
        openmc_rec_state = 2;
    //数据x高8位
    } else if (openmc_rec_state == 2) {
        rec_data_x = data << 8;
        openmc_rec_state = 3;
    //数据x低8位
    } else if (openmc_rec_state == 3) {
        rec_data_x |= data;
        openmc_rec_state = 4;
    //数据x高8位
    } else if (openmc_rec_state == 4) {
        rec_data_y = data << 8;
        openmc_rec_state = 5;
    //数据x低8位
    } else if (openmc_rec_state == 5) {
        rec_data_y |= data;
        switch (rec_type) {
            case 1:
                pole_distance = rec_data_x;
                break;
            case 2:
                line_high = rec_data_y;
                break;
        }
        openmv_updata_flag |= 1 << rec_type;
        openmc_rec_state = 0;
    } else {
        openmc_rec_state = 0;
    }
}
