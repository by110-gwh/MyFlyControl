#include "vl53l1x.h"
#include "VL53L1X_api.h"
#include "paramer_save.h"

#include "FreeRTOS.h"
#include "task.h"

//�߶�ԭʼ����
uint16_t high_raw_data;

/**********************************************************************************************************
*�� �� ��: vl53l1x_init
*����˵��: VL53L01X�������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void vl53l1x_init()
{
    uint8_t state;
    
    while (!state) {
        VL53L1X_BootState(0, &state);
        vTaskDelay(1);
    }
    VL53L1X_SensorInit(0);
    //VL53L1X_SetDistanceMode(0, 2);
    VL53L1X_SetInterMeasurementInMs(0, 100);
    //VL53L1X_SetTimingBudgetInMs(0, 100);
    //VL53L1X_SetROI(0, 16, 16);
    VL53L1X_StartRanging(0);
}

/**********************************************************************************************************
*�� �� ��: vl53l1x_calibration
*����˵��: VL53L01X������У׼
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void vl53l1x_calibration()
{
    
}

/**********************************************************************************************************
*�� �� ��: vl53l1x_task
*����˵��: VL53L01X��ȡ�߶ȣ��ɷ���������ã�Ƶ��200Hz
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void vl53l1x_task()
{
    static uint8_t state;
    uint8_t status;
    uint8_t data_ready;
    uint16_t distance;
    uint8_t range_status;
    if (state == 100 / 5) {
//        data_ready = 0;
//        while (1) {
//            status = VL53L1X_CheckForDataReady(0, &data_ready);
//            if (data_ready == 1)
//                break;
//            vTaskDelay(1);
//        }
        status = VL53L1X_GetRangeStatus(0, &range_status);
        status = VL53L1X_GetDistance(0, &distance);
        status = VL53L1X_ClearInterrupt(0);
        if (range_status == 0) {
            high_raw_data = distance;
            //printf("%d\r\n", distance);
        } else {
            high_raw_data = 0;
        }
        state = 0;
    }
    state++;
    
}
