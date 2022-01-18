/*####################################################################################
$$WISEGLOVESDK, Ver 1.3 beta
$$File name		:		WISEGLOVE.h
$$Compiler		:		VC6
$$Create date	:		2012-6 Ver beta
$$Revise date	:		2012-7 Ver 1.3.2
$$Revise date	:		2014-8 Ver 1.5.0
$$History		 	:
$$Auther		 	:
$$Powered By	:		Wiseglove Tech. Co., Ltd.	<www.wiseglove.com>
####################################################################################*/
#ifndef __WISEGLOVE_H__
#define __WISEGLOVE_H__

#ifdef DLL_IMPLEMENT  
#define DLL_API __declspec(dllexport)  
#else  
#define DLL_API __declspec(dllimport)  
#endif

/*
���ױ궨��־������SetCalibMode()����
*/
#define CALIB_AUTO  0
#define CALIB_MANU  1
typedef struct {
	float w;  //��Ԫ��
	float x;
	float y;
	float z;
} QUAT;

typedef struct {
	float x;
	float y;
	float z;
} EULER;


class DLL_API WiseGlove
{
public:

	/* ***************************************************
	*	���ܣ�ɨ��ϵͳ����ڵ�WISEGLOVE����
	*	��������
	*	���أ�WISEGLOVE������������
	*/
	virtual unsigned int Scan(void) = 0;
	/* ***************************************************
	*	���ܣ�ȡ�ô��ڶ˿ں�
	*	������num����ʾ��ϵͳ�еĵ�num�����׶˿�
	*	���أ�COM�ڶ˿ں�
	*/
	virtual unsigned int GetPort(unsigned int num) = 0;

	/* ***************************************************
	���ܣ�ȡ�����׵Ĵ���ͨѶ������
	������num����ʾ��ϵͳ�еĵ�num�����׶˿�
	���أ�����������115200��256000
	*/
	virtual unsigned int GetPortBaudrate(unsigned int num) = 0;
	/*  ***************************************************
	���ܣ���WISEGLOVE�˿�
	������nPort����ʾ���ڶ˿ں�
	���أ����سɹ�����TRUE(1)�� ���򷵻�FALSE(0)
	*/
	virtual bool  Open(unsigned int nPort, unsigned int baudrate) = 0;

	/*  ***************************************************
	���ܣ��ر�WG���ݶ˿�
	��������
	���أ���
	*/
	virtual	void Close(void) = 0;

	/* ***************************************************
	*	���ܣ�������ֻ֧������ָѹ�����е�WISEGLOVE�����ͺ�
	*	������char *filename���궨�ļ����ִ������硱c:\\calib.txt��
	*	���أ����سɹ�����TRUE(1)�� ���򷵻�FALSE(0)
	*/
	virtual bool LoadPressureCalib(char *filename) = 0;

	/* ***************************************************
	*	���ܣ���ʹ���ֶ��궨ʱ��ʹ�ñ��������ر궨�����ļ�
	*	������char *filename���궨�ļ����ִ������硱c:\\calib.txt��
	*	���أ����سɹ�����TRUE(1)�� ���򷵻�FALSE(0)
	*/
	virtual	bool LoadCalib(char *filename) = 0;

	/* ***************************************************
	*	���ܣ���ʹ���ֶ��궨������£�ʹ�ñ���������궨����
	*	������char *filename���궨�ļ����ִ������硱c:\\calib.txt��
	*	���أ����سɹ�����TRUE(1)�� ���򷵻�FALSE(0)
	*/
	virtual	bool SaveCalib(char *filename) = 0;

	/* ***************************************************
	*	���ܣ����ñ궨ģʽ
	*	������int mode
	*	mode = CALIB_AUTO�� �Զ��궨
	*	mode = 	CALIB_MANU�� �ֶ��궨
	*	���أ���
	*/
	virtual	void SetCalibMode(int mode) = 0;

	/* ***************************************************
	*	���ܣ����������ñ궨����
	*	������int index������������ֵ������19�������ͺŵķ�ΧΪ��0��18��
	*		  unsigned short min����Сֵ����
	*		  unsigned short max, ���ֵ����
	*	      float angle���ؽڻ�Ƕȷ�Χ����Ϊ��λ��
	*	���أ�����ɹ� ����TRUE(1)�����򷵻�FALSE(0)
	*/
	virtual bool SetCalib(int index, unsigned short min, unsigned short max, float angle) = 0;

	/* ***************************************************
	*	���ܣ�������������Сֵ����
	*	������int index������������ֵ����Χ��0��18��
	*		  unsigned short min����Сֵ����
	*	���أ�����ɹ� ����TRUE(1)�����򷵻�FALSE(0)
	*/
	virtual bool SetCalibmin(int index, unsigned short min) = 0;

	/* ***************************************************
	*	���ܣ��������������ֵ����
	*	������int index������������ֵ����Χ��0��18��
	*		  unsigned short max�����ֵ����
	*	���أ�����ɹ� ����TRUE(1)�����򷵻�FALSE(0)
	*/
	virtual bool SetCalibmax(int index, unsigned short max) = 0;

	/* ***************************************************
	*	���ܣ����������ùؽڻ�Ƕȷ�Χ����
	*	������int index������������ֵ����Χ��0��18��
	*		  float angle���ؽڻ�Ƕȷ�Χ
	*	���أ�����ɹ� ����TRUE(1)�����򷵻�FALSE(0)
	*/
	virtual bool SetCalibRange(int index, float angle) = 0;

	/* ***************************************************
	*	���ܣ�ȡ�ô�������Сֵ����ֵ
	*	������int index������������ֵ����Χ��0��18��
	*	���أ�����ɹ� ������Сֵ����ֵ�����򷵻�0
	*/
	virtual unsigned short GetCalibmin(int index) = 0;

	/* ***************************************************
	*	���ܣ�ȡ�ô��������ֵ����ֵ
	*	������int index������������ֵ����Χ��0��18��
	*	���أ�����ɹ� �������ֵ����ֵ�����򷵻�0
	*/
	virtual unsigned short GetCalibmax(int index) = 0;

	/* ***************************************************
	*	���ܣ�ȡ�ô������ؽڻ�Ƕȷ�Χ
	*	������int index������������ֵ����Χ��0��18��
	*	���أ�����ɹ� ���عؽڻ�Ƕȷ�Χ�����򷵻�0
	*/
	virtual float GetCalibRange(int index) = 0;

	/* ***************************************************
	*	���ܣ����ñ��������Իָ����б궨ֵ��ȱʡ״̬���궨ģʽ�ָ�Ϊ�Զ��궨ģʽ
	*	��������
	*	���أ���
	*/
	virtual	void ResetCalib() = 0;

	/* ***************************************************
	*	���ܣ����ô�������С�Ƕ�����
	*	������int index������������ֵ����Χ��0��18��
	*	      float min_adj����С�Ƕ�����ֵ����Χ��0��0.3��
	*	���أ�����ɹ� ����TRUE(1)�����򷵻�FALSE(0)
	*/
	virtual bool SetMin_adj(int index, float min_adj) = 0;

	/* ***************************************************
	*	���ܣ�����������С�Ƕ�����
	*	������int index������������
	*	���أ�����ɹ� ������С�Ƕ����ƣ����򷵻�0
	*/
	virtual float GetMin_adj(int index) = 0;

	/* ***************************************************
	*	���ܣ����ô��������Ƕ�����
	*	������int index������������ֵ����Χ��0��18��
	*	      float max_adj�����Ƕ�����ֵ����Χ��0��0.3��
	*	���أ�����ɹ� ����TRUE(1)�����򷵻�FALSE(0)
	*/
	virtual bool SetMax_adj(int index, float max_adj) = 0;

	/* ***************************************************
	*	���ܣ������������Ƕ�����
	*	������int index������������
	*	���أ�����ɹ� �������Ƕ����ƣ����򷵻�0
	*/
	virtual float GetMax_adj(int index) = 0;

	/* ***************************************************
	*	���ܣ��������������к��ִ�
	*	������unsigned char * sn���������к��ִ���һ������Ϊ16���ֽ�
	*	���أ�����ɹ� ����TRUE(1)�����򷵻�FALSE(0)
	*/
	virtual	bool GetSn(char *sn) = 0;

	/* ***************************************************
	*	���ܣ������������ͺ��ִ�
	*	������unsigned char * model�������ͺ��ִ���һ������Ϊ16���ֽ�
	*	���أ�����ɹ� ����TRUE(1)�����򷵻�FALSE(0)
	*/
	virtual	bool GetModel(char *model) = 0;

	/* ***************************************************
	*	���ܣ����������׳����ִ�
	*	������unsigned char * manu�����泧���ִ���һ������Ϊ16���ֽ�
	*	���أ�����ɹ� ����TRUE(1)�����򷵻�FALSE(0)
	*/
	virtual	bool GetManu(char *manu) = 0;

	/* ***************************************************
	*	���ܣ�����ָ��������������
	*	��������
	*	���أ�����ɹ� ���ش���������ֵ�����򷵻�0
	*/
	virtual int  GetNumOfSensor(void) = 0;

	/* ***************************************************
	*	���ܣ�����ָѹ������������
	*	��������
	*	���أ�����ɹ� ���ش���������ֵ�����򷵻�0
	*/
	virtual int  GetNumOfPressure(void) = 0;
	/* ***************************************************
	*	���ܣ����ֱ۴���������
	*	��������
	*	���أ�����ɹ� ���ش���������ֵ�����򷵻�0
	*/
	virtual int  GetNumOfArm(void) = 0;

	/* ***************************************************
	*	���ܣ������д�����ֵ
	*	������unsigned short * data�����������ݣ����19��unsigned short������
	*	���أ�����ɹ� ���ص�ǰ���ݰ���ʱ�������λ���룩�����򷵻�FALSE(0)
	*/
	//virtual	int GetData(unsigned short *data)=0;
	virtual	unsigned int GetData(unsigned short *data) = 0;

	/* ***************************************************
	*	���ܣ������д�������һ��ֵ����Χ��0��4095��
	*	������unsigned short * data�����������ݣ����19��unsigned short������
	*	���أ�����ɹ� ���ص�ǰ���ݰ���ʱ�������λ���룩�����򷵻�FALSE(0)
	*/
	//virtual	int GetScaledData(unsigned short *data)=0;
	virtual	unsigned int GetScaledData(unsigned short *data) = 0;

	/* ***************************************************
	*	���ܣ����������ĽǶ�
	*	������float *angle���������Ƕ����ݣ����19��float������
	*	���أ�����ɹ� ���ص�ǰ���ݰ���ʱ�������λ���룩�����򷵻�FALSE(0)
	*/
	virtual	unsigned int GetAngle(float *angle) = 0;

	/* ***************************************************
	*	���ܣ�����ָѹ����������ԭʼֵ
	*	������int *pressure��������ԭʼ���ݣ����5��int������
	*	���أ�����ɹ� ���ص�ǰ���ݰ���ʱ�������λ���룩�����򷵻�FALSE(0)
	*/
	virtual unsigned int GetPressureRaw(int *pressure) = 0;

	/* ***************************************************
	*	���ܣ�����ָѹ����������ѹ��
	*	������pressure��������ѹ�����ݣ���λ���ˣ����5��int������
	*	���أ�����ɹ� ���ص�ǰ���ݰ���ʱ�������λ���룩�����򷵻�FALSE(0)
	*/
	virtual unsigned int  GetPressure(int *pressure) = 0;

	/* ***************************************************
	*	���ܣ����ֱ۴���������ת�Ƕ�
	*	������float *quat��3���ֱ۴���������ת�Ƕȣ���ʽΪ��Ԫ��(w,x,y,z)
	*	���أ� ����ɹ� ����(1)�����򷵻�(0)
	*/
	virtual	unsigned int GetQuat(float *quat) = 0;

	/* ***************************************************
	*	���ܣ�������ָ������
	*	������unsigned char *fddata��5����ָ���������ݣ�����ΪĴָ��ʳָ����ָ������ָ��Сָ��ȡֵ��ΧΪ32-255
	*	���أ� ����ɹ� ����TRUE(1)�����򷵻�FALSE(0)
	*/
	virtual bool SetFeedBack(unsigned char *fddata) = 0;

	/* ***************************************************
	*	���ܣ��궨�ֱ۴������������������ʺ�UNITY3D
	*	�������ޡ�ע�⣬��������Ҫ��װ���ֱ���࣬���ֱۺ����Ʊ�����Ȼ�´�������
	*	���أ� ����ɹ� ����TRUE(1)�����򷵻�FALSE(0)
	*/
	virtual void ResetQuat() = 0;

	/* ***************************************************
	*	���ܣ����ûص���ŷ���Ƕȣ�Ϊ��һ�μ����ĽǶ�
	*	���������float* euler��ŷ����(x,y,z)
	*	���أ���
	*/
//	virtual void SetCallbackEuler(float* euler) = 0;

	/* ***************************************************
	*	���ܣ�����������Ԫ�ص�ŷ���Ƕ�
	*	���������float *quat ����Ԫ��(w,x,y,z); �������float *angle,������ŷ����(x,y,z)
	*	���أ���
	*/
//	virtual void GetEuler(float* quat, float *angle) = 0;

};

/* ***************************************************
 *	���ܣ������½�һ������ָ�룬�ڶ����׶˿ڲ���֮ǰ�����ȱ���Ҫ���ñ������½�һ������ָ�룬���磬WiseGlove* pGlove = GetWiseGlove();
 *	��������
 *	���أ�Wiseglove��ָ�루��ע�������κ�ʱ��ֻҪ�����˸ú���һ��Ҫִ��DelWiseGlove()����ָ�룬���������ڴ�й©��
 */
DLL_API WiseGlove* GetWiseGlove();
/* ***************************************************
 *	���ܣ���������һ������ָ��,ֻҪ���ù�GetWiseGlove()����������ָ�룬��ô�����˳�ǰ��һ��Ҫִ�б������ͷ���Դ�����������ڴ�й©��
 *	������WiseGlove* p��Wiseglove��ָ��
 *	���أ���
 */
DLL_API void DelWiseGlove(WiseGlove* p);
//=================================================================================
#endif  //end of wiseglove.h
