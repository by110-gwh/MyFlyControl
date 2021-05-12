#ifndef __USMART_STR_H
#define __USMART_STR_H	 

#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com 
//�汾��V3.3
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2011-2021
//All rights reserved
//********************************************************************************
//����˵��
//V1.4
//�����˶Բ���Ϊstring���͵ĺ�����֧��.���÷�Χ������.
//�Ż����ڴ�ռ��,��̬�ڴ�ռ��Ϊ79���ֽ�@10������.��̬��Ӧ���ּ��ַ�������
//V2.0 
//1,�޸���listָ��,��ӡ��������������ʽ.
//2,������idָ��,��ӡÿ����������ڵ�ַ.
//3,�޸��˲���ƥ��,֧�ֺ��������ĵ���(������ڵ�ַ).
//4,�����˺��������Ⱥ궨��.	
//V2.1 20110707		 
//1,����dec,hex����ָ��,�������ò�����ʾ����,��ִ�н���ת��.
//ע:��dec,hex����������ʱ��,���趨��ʾ��������.�����������ʱ��,��ִ�н���ת��.
//��:"dec 0XFF" ��Ὣ0XFFתΪ255,�ɴ��ڷ���.
//��:"hex 100" 	��Ὣ100תΪ0X64,�ɴ��ڷ���
//2,����usmart_get_cmdname����,���ڻ�ȡָ������.
//V2.2 20110726	
//1,������void���Ͳ����Ĳ���ͳ�ƴ���.
//2,�޸�������ʾ��ʽĬ��Ϊ16����.
//V2.3 20110815
//1,ȥ���˺�����������"("������.
//2,�������ַ��������в�����"("��bug.
//3,�޸��˺���Ĭ����ʾ������ʽ���޸ķ�ʽ. 
//V2.4 20110905
//1,�޸���usmart_get_cmdname����,������������������.����������������ʱ����������.
//2,����USMART_ENTIM2_SCAN�궨��,���������Ƿ�ʹ��TIM2��ʱִ��scan����.
//V2.5 20110930
//1,�޸�usmart_init����Ϊvoid usmart_init(uint8_t sysclk),���Ը���ϵͳƵ���Զ��趨ɨ��ʱ��.(�̶�100ms)
//2,ȥ����usmart_init�����е�uart_init����,���ڳ�ʼ���������ⲿ��ʼ��,�����û����й���.
//V2.6 20111009
//1,������read_addr��write_addr��������.��������������������д�ڲ������ַ(��������Ч��ַ).���ӷ������.
//2,read_addr��write_addr������������ͨ������USMART_USE_WRFUNSΪ��ʹ�ܺ͹ر�.
//3,�޸���usmart_strcmp,ʹ��淶��.			  
//V2.7 20111024
//1,�����˷���ֵ16������ʾʱ�����е�bug.
//2,�����˺����Ƿ��з���ֵ���ж�,���û�з���ֵ,�򲻻���ʾ.�з���ֵʱ����ʾ�䷵��ֵ.
//V2.8 20111116
//1,������list�Ȳ���������ָ��ͺ���ܵ���������bug.
//V2.9 20120917
//1,�޸������磺void*xxx(void)���ͺ�������ʶ���bug��
//V3.0 20130425
//1,�������ַ���������ת�����֧�֡�
//V3.1 20131120
//1,����runtimeϵͳָ��,��������ͳ�ƺ���ִ��ʱ��.
//�÷�:
//����:runtime 1 ,��������ִ��ʱ��ͳ�ƹ���
//����:runtime 0 ,��رպ���ִ��ʱ��ͳ�ƹ���
///runtimeͳ�ƹ���,��������:USMART_ENTIMX_SCAN Ϊ1,�ſ���ʹ��!!
//V3.2 20140828
//1,�޸�usmart_get_aparm����,����+/-���ŵ�֧��
//2,�޸�usmart_str2num����,֧�ָ���ת��
//V3.3 20160506
//����usmart_exe������USMART_ENTIMX_SCANΪ0��ʱ�򣬱�����bug
/////////////////////////////////////////////////////////////////////////////////////
  
uint8_t usmart_get_parmpos(uint8_t num);						//�õ�ĳ�������ڲ������������ʼλ��
uint8_t usmart_strcmp(uint8_t*str1,uint8_t *str2);					//�Ա������ַ����Ƿ����
uint32_t usmart_pow(uint8_t m,uint8_t n);							//M^N�η�
uint8_t usmart_str2num(uint8_t*str,uint32_t *res);					//�ַ���תΪ����
uint8_t usmart_get_cmdname(uint8_t*str,uint8_t*cmdname,uint8_t *nlen,uint8_t maxlen);//��str�еõ�ָ����,������ָ���
uint8_t usmart_get_fname(uint8_t*str,uint8_t*fname,uint8_t *pnum,uint8_t *rval);		//��str�еõ�������
uint8_t usmart_get_aparm(uint8_t *str,uint8_t *fparm,uint8_t *ptype); 	//��str�еõ�һ����������
uint8_t usmart_get_fparam(uint8_t*str,uint8_t *parn);  			//�õ�str�����еĺ�������.
#endif










