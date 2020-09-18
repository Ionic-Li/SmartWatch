#ifndef WIRELESS_H
#define WIRELESS_H
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

enum WirelessMode
{
	GEN_MODE, //一般模式
	WAKEUP_MODE, //唤醒模式
	POWSAVING_MODE,//省电模式
	SLEEP_MODE//睡眠模式
};

enum MsgType
{
	NONE,//无数据
	GPS_MSG,//GPS数据
	WAKEUP_MSG, //唤醒数据和消息确认消息
	LIGHTUP_MSG, //点亮LED
	NORMAL_MSG,//一般消息
	END
};
//使用串口1
class Wireless
{
public:
	//构造函数
	Wireless(unsigned int aux, unsigned int m0, unsigned int m1,
		uint8_t addr, uint8_t channel, WirelessMode mode = GEN_MODE);
	Wireless(Wireless& data) = delete;
	Wireless& operator=(const Wireless&) = delete;
	~Wireless();

	//设置模式
	void setMode(WirelessMode mode);
	//发送数据
	void sendMsg(MsgType type, const String& data);
	//发送数据
	void sendMsg(MsgType type, const char* data, int length);
	//接收消息
	MsgType getMsg(char** ppData, int* pLength);
	//接收消息
	MsgType getMsg(String& Data);
	//判断端口是否空闲
	inline bool available();
private:
	uint8_t TargetAddress;
	uint8_t TargetChannel;

	const unsigned int AUX;
	const unsigned int M0;
	const unsigned int M1;
	WirelessMode Mode;
};

bool Wireless::available()
{
	return digitalRead(AUX);
}

#endif