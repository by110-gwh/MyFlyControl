#include "openmv.h"

#include "FreeRTOS.h"
#include "queue.h"

//�˵����Ҿ���
volatile int16_t pole_distance;
//�����¾���
volatile int16_t line_high;
//���±�־
volatile uint32_t openmv_updata_flag;


/**********************************************************************************************************
*�� �� ��: openmv_init
*����˵��: openmv��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void openmv_init(void)
{
}

/**********************************************************************************************************
*�� �� ��: openmv_rec_callback
*����˵��: openmv�������ݻص�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void openmv_rec_callback(uint8_t data)
{
    static uint8_t openmc_rec_state;
    static uint8_t rec_type;
    static int16_t rec_data_x;
    static int16_t rec_data_y;
    //���յ�֡ͷ0xF3
    if (openmc_rec_state == 0 && data == 0xF3) {
        openmc_rec_state = 1;
    //��������
    } else if (openmc_rec_state == 1) {
        rec_type = data;
        openmc_rec_state = 2;
    //����x��8λ
    } else if (openmc_rec_state == 2) {
        rec_data_x = data << 8;
        openmc_rec_state = 3;
    //����x��8λ
    } else if (openmc_rec_state == 3) {
        rec_data_x |= data;
        openmc_rec_state = 4;
    //����x��8λ
    } else if (openmc_rec_state == 4) {
        rec_data_y = data << 8;
        openmc_rec_state = 5;
    //����x��8λ
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
