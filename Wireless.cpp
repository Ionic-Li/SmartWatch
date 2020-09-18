#include "Wireless.h"


Wireless::Wireless(unsigned int aux, unsigned int m0, unsigned int m1,
	uint8_t addr, uint8_t channel, WirelessMode mode)
	: AUX(aux), M0(m0), M1(m1)
{
	TargetAddress = addr;
	TargetChannel = channel;
	pinMode(M0, OUTPUT);
	pinMode(M1, OUTPUT);
	
	setMode(mode);
	Serial1.begin(9600);
}

Wireless::~Wireless()
{
}

void Wireless::setMode(WirelessMode mode)
{
	Mode = mode;
	switch (Mode)
	{
	case GEN_MODE:
		digitalWrite(M0, LOW);
		digitalWrite(M1, LOW);
		break;
	case WAKEUP_MODE:
		digitalWrite(M0, HIGH);
		digitalWrite(M1, LOW);
		break;
	case POWSAVING_MODE:
		digitalWrite(M0, LOW);
		digitalWrite(M1, HIGH);
		break;
	case SLEEP_MODE:
		digitalWrite(M0, HIGH);
		digitalWrite(M1, HIGH);
		break;
	}
}

void Wireless::sendMsg(MsgType type, const String & data)
{
	sendMsg(type, data.c_str(), data.length());
}

void Wireless::sendMsg(MsgType type, const char * pData, int length)
{
	int Size = 3 + 2 * sizeof(int) + length;
	char* ToSendData = new char[Size];
	ToSendData[0] = 0x00;
	ToSendData[1] = TargetAddress;
	ToSendData[2] = TargetChannel;
	*(int*)(ToSendData + 3) = type;
	*(int*)(ToSendData + 3 + sizeof(int)) = length;
	memcpy(ToSendData + 3 + 2 * sizeof(int), pData, length);
	if (available())//避免出现消息阻塞
	{
		Serial1.write(ToSendData, Size);
	}
	delete[] ToSendData;
}

MsgType Wireless::getMsg(char** ppData, int* pLength)
{
	MsgType RecType = NONE;
	*pLength = 0;

	if (Serial1.available())
	{
		Serial1.readBytes((char*)&RecType, sizeof(int));
		if (RecType > NONE && RecType < END)//如果消息有效
		{
			Serial1.readBytes((char*)pLength, sizeof(int));
			*ppData = new char[*pLength];//临时缓冲区
			Serial1.readBytes(*ppData, *pLength);//存放数据
		}	
		else
		{
			RecType = NONE;//如果消息无效
		}
	}
	return RecType;
}

MsgType Wireless::getMsg(String& Data)
{
	Data = String();//清空数据
	char* pTempData;
	int Length = 0;
	MsgType RecType = getMsg(&pTempData, &Length);
	if (Length != 0)
	{
		char* pTemp = new char[Length + 1];
		memset(pTemp, 0, Length + 1);
		memcpy(pTemp, pTempData, Length);
		Data = String(pTemp);
		delete[] pTemp;
		delete[] pTempData;
	}
	return RecType;
}
