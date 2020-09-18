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

const int VALCNT = 7;//一次读取寄存器的数量
const int CALIBTIMES = 1000; //校准时读数的次数
const int MPU = 0x68; //MPU-6050的I2C地址

const float RAD2DEG = 57.295779513f; //将弧度转为角度的乘数

class MPU6050
{
public:
	//构造函数
	MPU6050();
	//析构函数
	~MPU6050();

	void begin();
	//从MPU6050读出加速度计三个分量、温度和三个角速度计的原时数据
	//保存在指定的数组中
	void readAccGyrRaw(int *pVals);
	//算得Roll角原始数据（角度制）
	float getRollRaw(float *pRealVals, float fNorm);
	//算得Pitch角的原时数据（角度制）
	float getPitchRaw(float *pRealVals, float fNorm);
	//获得滤波后的Roll和Pitch角和两次测量间平均角速度[row, pitch, rowRate, pitchRate]（弧度制）
	void getAngleAndRate(float *pReadOuts);
	//对读数进行纠正，消除偏移，并转换为物理量
	void rectify(int *pReadout, float *pRealVals);
	
private:
	//向MPU6050写入一个字节的数据
	//指定寄存器地址与一个字节的值
	void writeMPUReg(int nReg, unsigned char nVal);
	//从MPU6050读出一个字节的数据
	//指定寄存器地址，返回读出的值
	unsigned char readMPUReg(int nReg);

	//对大量读数进行统计，校准平均偏移量
	void calibration();

	int CalibData[VALCNT]; //校准数据
	unsigned long uLastTime;
	float fLastRoll; //上一次滤波得到的Roll角
	float fLastPitch; //上一次滤波得到的Pitch角
	Kalman KalmanRoll; //Roll角滤波器
	Kalman KalmanPitch; //Pitch角滤波器
};


#endif