#ifndef WIRELESS_H
#define WIRELESS_H
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

enum WirelessMode
{
	GEN_MODE, //һ��ģʽ
	WAKEUP_MODE, //����ģʽ
	POWSAVING_MODE,//ʡ��ģʽ
	SLEEP_MODE//˯��ģʽ
};

enum MsgType
{
	NONE,//������
	GPS_MSG,//GPS����
	WAKEUP_MSG, //�������ݺ���Ϣȷ����Ϣ
	LIGHTUP_MSG, //����LED
	NORMAL_MSG,//һ����Ϣ
	END
};
//ʹ�ô���1
class Wireless
{
public:
	//���캯��
	Wireless(unsigned int aux, unsigned int m0, unsigned int m1,
		uint8_t addr, uint8_t channel, WirelessMode mode = GEN_MODE);
	Wireless(Wireless& data) = delete;
	Wireless& operator=(const Wireless&) = delete;
	~Wireless();

	//����ģʽ
	void setMode(WirelessMode mode);
	//��������
	void sendMsg(MsgType type, const String& data);
	//��������
	void sendMsg(MsgType type, const char* data, int length);
	//������Ϣ
	MsgType getMsg(char** ppData, int* pLength);
	//������Ϣ
	MsgType getMsg(String& Data);
	//�ж϶˿��Ƿ����
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