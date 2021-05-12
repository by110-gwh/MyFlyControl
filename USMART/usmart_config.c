#include "usmart.h"
#include "usmart_str.h"

		void angle_control_pid_set(uint8_t p, uint8_t i, uint8_t d);
		void gyro_control_pid_set(uint8_t p, uint8_t i, uint8_t d);
		void HAL_Delay(uint32_t Delay);
//�������б��ʼ��(�û��Լ����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�
struct _m_usmart_nametab usmart_nametab[] = {
//���ʹ���˶�д����
#if USMART_USE_WRFUNS==1
	(void*)read_addr,"u32 read_addr(u32 addr)",
	(void*)write_addr,"void write_addr(u32 addr,u32 val)",
	(void*)HAL_Delay,"void HAL_Delay(uint32_t Delay)",
	(void*)angle_control_pid_set,"void angle_control_pid_set(uint8_t p, uint8_t i, uint8_t d)",
	(void*)gyro_control_pid_set,"void gyro_control_pid_set(uint8_t p, uint8_t i, uint8_t d)"

#endif		   
							
};						  

//�������ƹ�������ʼ���õ������ܿغ��������ֵõ�����������
struct _m_usmart_dev usmart_dev = {
	usmart_nametab,
	usmart_init,
	usmart_cmd_rec,
	usmart_exe,
	usmart_scan,
	//��������
	sizeof(usmart_nametab)/sizeof(struct _m_usmart_nametab),
	//��������
	0,
	//����ID
	0,
	//������ʾ����,0,10����;1,16����
	1,
	//��������.bitx:,0,����;1,�ַ���
	0,
	//ÿ�������ĳ����ݴ��,��ҪMAX_PARM��0��ʼ��
	0,
	//�����Ĳ���,��ҪPARM_LEN��0��ʼ��
	0,
};   



















