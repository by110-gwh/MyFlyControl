#include "usmart.h"
#include "usmart_str.h"

//�������б��ʼ��(�û��Լ����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�
struct _m_usmart_nametab usmart_nametab[] = {
//���ʹ���˶�д����
#if USMART_USE_WRFUNS==1
	(void*)read_addr,"u32 read_addr(u32 addr)",
	(void*)write_addr,"void write_addr(u32 addr,u32 val)",

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



















