// MPU6050.h

#ifndef _MPU6050_h
#define _MPU6050_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Kalman.h>
#include <Wire.h>
#include <Math.h>

const int VALCNT = 7;//һ�ζ�ȡ�Ĵ���������
const int CALIBTIMES = 1000; //У׼ʱ�����Ĵ���
const int MPU = 0x68; //MPU-6050��I2C��ַ

const float RAD2DEG = 57.295779513f; //������תΪ�Ƕȵĳ���

class MPU6050
{
public:
	//���캯��
	MPU6050();
	//��������
	~MPU6050();

	void begin();
	//��MPU6050�������ٶȼ������������¶Ⱥ��������ٶȼƵ�ԭʱ����
	//������ָ����������
	void readAccGyrRaw(int *pVals);
	//���Roll��ԭʼ���ݣ��Ƕ��ƣ�
	float getRollRaw(float *pRealVals, float fNorm);
	//���Pitch�ǵ�ԭʱ���ݣ��Ƕ��ƣ�
	float getPitchRaw(float *pRealVals, float fNorm);
	//����˲����Roll��Pitch�Ǻ����β�����ƽ�����ٶ�[row, pitch, rowRate, pitchRate]�������ƣ�
	void getAngleAndRate(float *pReadOuts);
	//�Զ������о���������ƫ�ƣ���ת��Ϊ������
	void rectify(int *pReadout, float *pRealVals);
	
private:
	//��MPU6050д��һ���ֽڵ�����
	//ָ���Ĵ�����ַ��һ���ֽڵ�ֵ
	void writeMPUReg(int nReg, unsigned char nVal);
	//��MPU6050����һ���ֽڵ�����
	//ָ���Ĵ�����ַ�����ض�����ֵ
	unsigned char readMPUReg(int nReg);

	//�Դ�����������ͳ�ƣ�У׼ƽ��ƫ����
	void calibration();

	int CalibData[VALCNT]; //У׼����
	unsigned long uLastTime;
	float fLastRoll; //��һ���˲��õ���Roll��
	float fLastPitch; //��һ���˲��õ���Pitch��
	Kalman KalmanRoll; //Roll���˲���
	Kalman KalmanPitch; //Pitch���˲���
};


#endif