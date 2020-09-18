// 
// 
// 

#include "MPU6050.h"

MPU6050::MPU6050()
{
}

MPU6050::~MPU6050()
{
}

void MPU6050::begin()
{
	Wire.begin(); //初始化Wire库
	writeMPUReg(0x6B, 0); //启动MPU6050设备

	calibration(); //执行校准
	uLastTime = micros(); //记录当前时间
	fLastRoll = 0.0f;
	fLastPitch = 0.0f;
}


void MPU6050::readAccGyrRaw(int * pVals)
{
	Wire.beginTransmission(MPU);
	Wire.write(0x3B);
	Wire.requestFrom(MPU, VALCNT * 2, true);
	Wire.endTransmission(true);
	for (long i = 0; i < VALCNT; ++i) {
		pVals[i] = Wire.read() << 8 | Wire.read();
	}
}

float MPU6050::getRollRaw(float * pRealVals, float fNorm)
{
	float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
	float fCos = fNormXZ / fNorm;
	return acos(fCos) * RAD2DEG;
}

float MPU6050::getPitchRaw(float * pRealVals, float fNorm)
{
	float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
	float fCos = fNormYZ / fNorm;
	return acos(fCos) * RAD2DEG;
}

void MPU6050::getAngleAndRate(float * pReadOuts)
{
	int Readouts[VALCNT];
	readAccGyrRaw(Readouts); //读出测量值

	float RealVals[7];
	rectify(Readouts, RealVals); //根据校准的偏移量进行纠正

	//计算加速度向量的模长，均以g为单位
	float fNorm = sqrt(RealVals[0] * RealVals[0] + RealVals[1] * RealVals[1] + RealVals[2] * RealVals[2]);
	float fRoll = getRollRaw(RealVals, fNorm); //计算Roll角
	if (RealVals[1] > 0) {
		fRoll = -fRoll;
	}
	float fPitch = getPitchRaw(RealVals, fNorm); //计算Pitch角
	if (RealVals[0] < 0) {
		fPitch = -fPitch;
	}

	//计算两次测量的时间间隔dt，以秒为单位
	unsigned long uCurTime = micros();
	float dt = (double)(uCurTime - uLastTime) / 1000000.0;
	//对Roll角和Pitch角进行卡尔曼滤波
	float fNewRoll = KalmanRoll.getAngle(fRoll, RealVals[4], dt) / RAD2DEG;
	float fNewPitch = KalmanPitch.getAngle(fPitch, RealVals[5], dt) / RAD2DEG;
	//跟据滤波值计算角度速
	float fRollRate = (fNewRoll - fLastRoll) / dt;
	float fPitchRate = (fNewPitch - fLastPitch) / dt;
	
	//更新Roll角和Pitch角
	fLastRoll = fNewRoll;
	fLastPitch = fNewPitch;
	//更新本次测的时间
	uLastTime = uCurTime;

	pReadOuts[0] = fNewRoll;
	pReadOuts[1] = fNewPitch;
	pReadOuts[2] = fRollRate;
	pReadOuts[3] = fPitchRate;
}

void MPU6050::writeMPUReg(int nReg, unsigned char nVal)
{
	Wire.beginTransmission(MPU);
	Wire.write(nReg);
	Wire.write(nVal);
	Wire.endTransmission(true);
}

unsigned char MPU6050::readMPUReg(int nReg)
{
	Wire.beginTransmission(MPU);
	Wire.write(nReg);
	Wire.requestFrom(MPU, 1, true);
	Wire.endTransmission(true);
	return Wire.read();
}

void MPU6050::calibration()
{
	float ValSums[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };
	//先求和
	for (int i = 0; i < CALIBTIMES; ++i) {
		int MpuVals[VALCNT];
		readAccGyrRaw(MpuVals);
		for (int j = 0; j < VALCNT; ++j) {
			ValSums[j] += MpuVals[j];
		}
	}
	//再求平均
	for (int i = 0; i < VALCNT; ++i) {
		CalibData[i] = int(ValSums[i] / CALIBTIMES);
	}
	CalibData[2] += 16384; //设芯片Z轴竖直向下，设定静态工作点。
}

void MPU6050::rectify(int * pReadout, float * pRealVals)
{
	for (int i = 0; i < 3; ++i) {
		pRealVals[i] = (float)(pReadout[i] - CalibData[i]) / 16384.0f;
	}
	pRealVals[3] = pReadout[3] / 340.0f + 36.53;
	for (int i = 4; i < 7; ++i) {
		pRealVals[i] = (float)(pReadout[i] - CalibData[i]) / 131.0f;
	}
}