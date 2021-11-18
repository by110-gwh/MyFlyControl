#include "openmv.h"

#include "FreeRTOS.h"
#include "queue.h"

//�����ľ�������
volatile int16_t land_x;
volatile int16_t land_y;
//�ߵľ�������
volatile int16_t line_fl;
volatile int16_t line_fr;
volatile int16_t line_rf;
volatile int16_t line_rb;
volatile int16_t line_lf;
volatile int16_t line_lb;
volatile int16_t line_bl;
volatile int16_t line_br;
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
    //����y��8λ
    } else if (openmc_rec_state == 4) {
        rec_data_y = data << 8;
        openmc_rec_state = 5;
    //����y��8λ
    } else if (openmc_rec_state == 5) {
        rec_data_y |= data;
        switch (rec_type) {
            case 1:
                land_x = rec_data_x;
                land_y = rec_data_y;
                break;
            case 3:
                if (rec_data_x != -80) {
                    line_lb = rec_data_x;
                    openmv_updata_flag |= 1 << 20;
                }
                if (rec_data_x != -60) {
                    line_bl = rec_data_y;
                    openmv_updata_flag |= 1 << 21;
                }
                break;
            case 4:
                if (rec_data_x != 80) {
                    line_lf = rec_data_x;
                    openmv_updata_flag |= 1 << 22;
                }
                if (rec_data_y != -60) {
                    line_fl = rec_data_y;
                    openmv_updata_flag |= 1 << 23;
                }
                break;
            case 5:
                if (rec_data_x != -80) {
                    line_rb = rec_data_x;
                    openmv_updata_flag |= 1 << 24;
                }
                if (rec_data_y != 60) {
                    line_br = rec_data_y;
                    openmv_updata_flag |= 1 << 25;
                }
                break;
            case 6:
                if (rec_data_x != 80) {
                    line_rf = rec_data_x;
                    openmv_updata_flag |= 1 << 26;
                }
                if (rec_data_y != 60) {
                    line_fr = rec_data_y;
                    openmv_updata_flag |= 1 << 27;
                }
                break;
        }
        openmv_updata_flag |= 1 << rec_type;
        openmc_rec_state = 0;
    } else {
        openmc_rec_state = 0;
    }
}
