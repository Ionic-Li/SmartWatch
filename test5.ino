/*
   Name:	test5.ino
   Created:	2020/7/10
   Author:	lyj01
*/
#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Bounce2.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <HMC5883L.h>
#include "MPU6050.h"
#include "Wireless.h"

//the states of fsm
enum States
{
	CLOCK_STATE,
	COMPASS_STATE,//compass mode
	NAVIGATION_STATE,//navigation mode
};

//PIN definations
const uint8_t BTN_PIN = 2;
const uint8_t REC_PIN = 32;
const uint8_t PL_PIN = 34;

//Obj definations
HMC5883L Compass;
MPU6050 Mpu;
U8G2_SSD1306_128X64_NONAME_1_HW_I2C  u8g2(U8G2_R0);//OLED
Bounce Btn;
TinyGPSPlus GPS;//Serial2 is GPS serial
//aux = 23, m0 = 24, m1 = 25, target address is 0x05, target channel is 0x09
Wireless Emitter(23, 24, 25, 5, 9, SLEEP_MODE);

//constants
const float DECLINATIONANGLE = -(7.0 + (13.0 / 60.0)) / (180 / M_PI);//declination angle

//variable defination
uint8_t OldState = HIGH;//used to memorize the old state of the button
bool BtnFlag = false;
unsigned long BeginTime = 0;//used to record the begining time when the btn has been pressed
//unsigned long LastMsgTime = 0;//used to determine how long a Msg hasn't been received(in Navigation mode)
float Angle;//the angle measures by the compass
States FsmState = CLOCK_STATE;//current state
double DesLongitude = 116.38;//the longitude of the destination
double DesLatitude = 39.90;//the latitude of the destination
double Distance = -1;//the distance to the destination, measures in meter
double Course = -1;//the course to the destination, measures in RAD
uint16_t Year = -1;//date
uint8_t Month = -1;
uint8_t Day = -1;
String CurrDate = "";//used for OLED display
uint8_t Hour = -1;//time
uint8_t Minute = -1;
volatile uint8_t Second = -1;
String CurrTime = "";//used for OLED display
uint8_t LastDay = 0;//the last day of the month
volatile uint16_t Counter1 = 0;//used in the interrupt function to get time
volatile uint8_t Counter2 = 0;//used in the interrupt function to return to clock mode
unsigned int CountStep = 0;//used to count steps
float HighAcc = 0;
unsigned int LastStepTime = 0;
bool LightupFlag = false;

//function declaration
float getAngle();//get direction angle, measures in RAD
void setState(States NewState);//set fsm state
void changeState();//change the state of fsm
void updateData();//update the distance and course to the destination
void getMsg();//get messages from the target
inline int getTimeZone(double Lat, double Lng);//get current time zone
void getLocalTime();//get local time
void adjustTime();//adjust time to the right format
inline void setCurrTime();//set current time string for display
inline void setCurrDate();//set current date string for display
void interrupt();//attached to Timer1
void btnInterrupt();//when the btn has been pressed
inline void resetState();//reset state to clock
void updateStep();//update step data
void draw();//called by u8g2
//different draw function for different state
inline void drawClock();//ClockState
inline void drawNav();//Navigation state
inline void drawCompass();//CompassState

//-------------------------------setup and loop--------------------------------

//initialize
void setup()
{
	Wire.begin();
	Serial.begin(9600);

	//btn settings
	pinMode(BTN_PIN, INPUT_PULLUP);
	Btn.attach(BTN_PIN);
	Btn.interval(20);

	//u8g2 initialize
	u8g2.begin();

	//Mpu initialize
	Mpu.begin();

	//Compass initialize
	Serial.println("Initialize HMC5883L");
	while (!Compass.begin())
	{
		Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
		delay(500);
	}
	Compass.setRange(HMC5883L_RANGE_1_3GA);
	// Set measurement mode
	Compass.setMeasurementMode(HMC5883L_CONTINOUS);
	// Set data rate
	Compass.setDataRate(HMC5883L_DATARATE_30HZ);
	// Set number of samples averaged
	Compass.setSamples(HMC5883L_SAMPLES_8);
	// Set calibration offset
	Compass.setOffset(-102, -202, -69);

	//record and play initialize
	pinMode(REC_PIN, OUTPUT);
	pinMode(PL_PIN, OUTPUT);

	//gps serial initialize
	Serial2.begin(9600);
	//get current time
	while (-1 == Year)
	{
		getLocalTime();
	}

	//Timer1 initialize
	Timer1.initialize();
	Timer1.attachInterrupt(interrupt);

	attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnInterrupt, LOW);
}

//main loop
void loop()
{
	//picture loop
	u8g2.firstPage();
	do
	{
		draw();
	} while (u8g2.nextPage());

	if (CLOCK_STATE != FsmState)
	{
		//get the state of the btn
		Btn.update();
		uint8_t CurState = Btn.read();//get current button state
		if (LOW == CurState && HIGH == OldState)//if the btn has been pressed
		{
			BeginTime = millis();
		}
		else if (LOW == OldState && HIGH == CurState)//if the btn has been released, determine whether it's long or short press
		{
			unsigned long Interval = millis() - BeginTime;//count the interval
			//if the btn as been pressed for over 850ms, it will be considered as long press
			//if FsmState is navigation, send a signal to light up the LED
			if (Interval >= 850 && NAVIGATION_STATE == FsmState)
			{
				//wait for the Eimmiter to be available and send a lightup message
				while (!Emitter.available());
				Emitter.sendMsg(LIGHTUP_MSG, "");
				Counter2 = 0;
				Serial.println(1);
			}
			//if the btn has been pressed for less than 850ms, it will be considered as short press
			//the FsmState will be changed
			else if (Interval < 850)
			{
				changeState();
			}
		}
		OldState = CurState;
	}
	else if (BtnFlag)
	{
		changeState();
	}

	if (Counter1 >= 3600)//reset time every hour
	{
		Counter1 = 0;
		getLocalTime();
	}
	adjustTime();//set time to the normal format

	updateStep();//update step

	//FSM
	switch (FsmState)
	{
	case CLOCK_STATE:
		break;
	case COMPASS_STATE:
		Angle = getAngle();
		resetState();
		//Serial.print(Angle);
		break;
	case NAVIGATION_STATE:
		getMsg();
		//if target address has been received
		if (100 != DesLatitude)
		{
			//update angle and gps data
			Angle = getAngle();
			updateData();
		}
		//ensure the connetion is still on
		if (Counter2 % 6 == 5)
		{
			Emitter.sendMsg(WAKEUP_MSG, "");
			Counter2 = 0;
		}
		////if gps data hasn't been received for 3 minutes set the destination location invalid
		//if (millis() - LastMsgTime >= 180000)
		//{
		//	DesLatitude = 100;
		//	Distance = -1;
		//}
		break;
	}
}

//-------------------------function defination---------------------------------

//return the direction angle, measures in RAD(calibrated with the attitude angle)
float getAngle()
{
	//get roll and pitch(in RAD)
	float Readouts[4];
	Mpu.getAngleAndRate(Readouts);
	//roll and pitch in the coordinates of HMC5883L
	float Pitch = -Readouts[0];
	float Roll = -Readouts[1];
	//read the strength of the magnetic fields
	Vector Norm = Compass.readNormalize();
	//calculate the calibrated angle
	float Xh = Norm.XAxis * cos(Pitch) + Norm.YAxis * sin(Roll) * sin(Pitch)
		- Norm.ZAxis * cos(Roll) * sin(Pitch);
	float Yh = Norm.YAxis * cos(Roll) + Norm.ZAxis * sin(Roll);

	float Heading = atan2(Yh, Xh) + PI / 2;
	//add the declination angle
	Heading += DECLINATIONANGLE;
	if (Heading < 0)
	{
		Heading += 2 * PI;
	}

	if (Heading > 2 * PI)
	{
		Heading -= 2 * PI;
	}
	return Heading;
}

void setState(States NewState)
{
	//if state hasn't been changed, return
	if (FsmState == NewState)
	{
		return;
	}
	//if state has been changed, cleanup the old state and enter the new one
	switch (FsmState)
	{
	case CLOCK_STATE:
		detachInterrupt(digitalPinToInterrupt(BTN_PIN));
		break;
	case COMPASS_STATE:
		break;
	case NAVIGATION_STATE:
		//set the emitter to sleep mode to save power
		Emitter.setMode(SLEEP_MODE);
		break;
	}
	switch (NewState)
	{
	case CLOCK_STATE:
		attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnInterrupt, LOW);
		BtnFlag = false;
		break;
	case COMPASS_STATE:
		Counter2 = 0;
		break;
	case NAVIGATION_STATE:
		Emitter.setMode(WAKEUP_MODE);
		Distance = -1;
		DesLatitude = 100;
		//LastMsgTime = 0;
		Counter2 = 0;
		LightupFlag = false;
		break;
	}
	FsmState = NewState;
}

void changeState()
{
	switch (FsmState)
	{
	case CLOCK_STATE:
		setState(COMPASS_STATE);
		break;
	case COMPASS_STATE:
		setState(NAVIGATION_STATE);
		break;
	case NAVIGATION_STATE:
		setState(CLOCK_STATE);
		break;
	}
}

void updateData()
{
	//get gps data
	while (Serial2.available())
	{
		GPS.encode(Serial2.read());
	}
	//update data
	if (GPS.location.isUpdated() && DesLatitude != 100)
	{
		Distance = TinyGPSPlus::distanceBetween(DesLatitude, DesLongitude,
			GPS.location.lat(), GPS.location.lng());
		Course = TinyGPSPlus::courseTo(GPS.location.lat(), GPS.location.lng(),
			DesLatitude, DesLongitude) * PI / 180;
		Serial.println(String(DesLatitude, 7));
		Serial.println(String(DesLongitude, 7));
		Serial.println(String(GPS.location.lat(), 7));
		Serial.println(String(GPS.location.lng(), 7));
		Serial.println(0);
		if (Distance <= 10 && !LightupFlag)
		{
			while (!Emitter.available());
			Emitter.sendMsg(LIGHTUP_MSG, "");
			LightupFlag = true;
		}
		if (Distance > 10)
		{
			LightupFlag = false;
		}
	}
}

void getMsg()
{
	char* MsgData;
	int MsgLength = 0;
	//get message
	MsgType Type = Emitter.getMsg(&MsgData, &MsgLength);
	switch (Type)
	{
	case GPS_MSG:
		if (MsgLength > 0)
		{
			if (MsgLength == 2 * sizeof(double))
			{
				//get the location of destination
				DesLatitude = ((double*)MsgData)[0];
				DesLongitude = ((double*)MsgData)[1];
				//LastMsgTime = millis();
			}
		}
		break;
	default:
		break;
	}
	//if receives data, delete it
	if (MsgLength > 0)
	{
		delete[] MsgData;
	}
}

int getTimeZone(double Lat, double Lng)
{
	//if in China time zone is 8
	if ((Lat >= 17.9 && Lat <= 53 && Lng >= 75 && Lng <= 125) || (Lat >= 40 && Lat <= 53 && Lng >= 125 && Lng <= 135))
	{
		return 8;
	}
	bool Negative = false;//east
	if (Lng < 0)//west
	{
		Negative = true;
		Lng = -Lng;
	}
	int TimeZone = (int)(Lng / 15 + 0.5);
	if (Negative)
	{
		TimeZone = -TimeZone;
	}
	return TimeZone;
}

void getLocalTime()
{
	while (Serial2.available())
	{
		GPS.encode(Serial2.read());
	}
	//get local time
	if (GPS.date.isUpdated() && GPS.time.isUpdated() && GPS.location.isUpdated())
	{
		Year = GPS.date.year();
		Month = GPS.date.month();
		Day = GPS.date.day();
		Hour = GPS.time.hour() + getTimeZone(GPS.location.lat(), GPS.location.lng());
		Minute = GPS.time.minute();
		Second = GPS.time.second();

		uint8_t PreLastDay = 0;
		uint8_t NextLastDay = 0;
		if (Month == 1 || Month == 3 || Month == 5 || Month == 7 || Month == 8 || Month == 10 || Month == 12)
		{
			LastDay = 31;
			if (3 == Month)
			{
				//whether is leap year
				if ((Year % 400 == 0) || (Year % 4 == 0 && Year % 100 != 0))
				{
					PreLastDay = 29;
				}
				else
				{
					PreLastDay = 28;
				}
				NextLastDay = 30;
			}
			else if (1 == Month)
			{
				//whether is leap year
				if ((Year % 400 == 0) || (Year % 4 == 0 && Year % 100 != 0))
				{
					NextLastDay = 29;
				}
				else
				{
					NextLastDay = 28;
				}
				PreLastDay = 31;
			}
			else if (8 == Month)
			{
				PreLastDay = 31;
				NextLastDay = 30;
			}
			else if (7 == Month && 12 == Month)
			{
				PreLastDay = 30;
				NextLastDay = 31;
			}
			else
			{
				PreLastDay = 30;
				NextLastDay = 30;
			}
		}
		else if (Month == 4 || Month == 6 || Month == 9 || Month == 11)
		{
			LastDay = 30;
			PreLastDay = 31;
			NextLastDay = 31;
		}
		//February
		else
		{
			PreLastDay = 31;
			//whether is leap year
			if ((Year % 400 == 0) || (Year % 4 == 0 && Year % 100 != 0))
			{
				LastDay = 29;
			}
			else
			{
				LastDay = 28;
			}
			NextLastDay = 31;
		}

		//calculate the time and date
		if (Hour >= 24)
		{
			Hour -= 24;
			Day += 1;

			if (Day > LastDay)//next month
			{
				Day -= LastDay;
				Month += 1;
				
				if (Month > 12)//next year
				{
					Month -= 12;
					Year += 1;
				}

				LastDay = NextLastDay;
			}
		}
		if (Hour < 0)
		{
			Hour += 24;
			Day -= 1;
			if (Day < 1)//last month
			{
				Day = PreLastDay;
				Month -= 1;
				if (Month < 1)//last year
				{
					Month = 12;
					Year -= 1;
				}
				LastDay = PreLastDay;
			}
		}
		setCurrDate();
		setCurrTime();
	}
}

void adjustTime()
{
	if (Second >= 60)
	{
		Second -= 60;
		Minute += 1;
		if (Minute >= 60)
		{
			Minute -= 60;
			Hour += 1;
		}
		if (Hour >= 24)
		{
			Hour -= 24;
			Day += 1;

			CountStep = 0;//refresh step

			if (Day > LastDay)//next month
			{
				Day -= LastDay;
				Month += 1;

				if (Month > 12)//next year
				{
					Month -= 12;
					Year += 1;
				}

				if (Month == 1 || Month == 3 || Month == 5 || Month == 7 || Month == 8 || Month == 10 || Month == 12)
				{
					LastDay = 31;
				}
				else if (Month == 4 || Month == 6 || Month == 9 || Month == 11)
				{
					LastDay = 30;
				}
				//February
				else
				{
					//whether is leap year
					if ((Year % 400 == 0) || (Year % 4 == 0 && Year % 100 != 0))
					{
						LastDay = 29;
					}
					else
					{
						LastDay = 28;
					}
				}
			}
			setCurrDate();
		}
	}
	setCurrTime();
}

void setCurrTime()
{
	//clear the current time string
	CurrTime = "";
	//display time as hh:mm:ss
	if (Hour < 10)
	{
		CurrTime += 0;
	}
	CurrTime += Hour;
	CurrTime += ':';
	if (Minute < 10)
	{
		CurrTime += 0;
	}
	CurrTime += Minute;
	CurrTime += ':';
	if (Second < 10)
	{
		CurrTime += 0;
	}
	CurrTime += Second;
}

void setCurrDate()
{
	//clear current date string
	CurrDate = "";
	CurrDate += Year;
	CurrDate += '/';
	if (Month < 10)
	{
		CurrDate += 0;
	}
	CurrDate += Month;
	CurrDate += '/';
	if (Day < 10)
	{
		CurrDate += 0;
	}
	CurrDate += Day;
}

void interrupt()
{
	Counter1++;
	Second++;
	switch (FsmState)
	{
	case CLOCK_STATE:
		break;
	default:
		Counter2++;
		break;
	}
}

void btnInterrupt()
{
	BtnFlag = true;
}

inline void resetState()
{
	//return to clock state every 60 second
	if (Counter2 >= 60)
	{
		setState(CLOCK_STATE);
		Counter2 = 0;//reset counter to 0
	}
}

void updateStep()
{
	int Readouts[VALCNT];
	Mpu.readAccGyrRaw(Readouts);

	float RealVals[7];
	Mpu.rectify(Readouts, RealVals);

	float QuaAcc = sqrt(RealVals[0] * RealVals[0] + RealVals[1] * RealVals[1] + RealVals[2] * RealVals[2]);

	if (QuaAcc > HighAcc)
	{
		HighAcc = QuaAcc;
	}
	else if (HighAcc - QuaAcc > 0.16)
	{
		float CurrTime = millis();
		if (CurrTime - LastStepTime >= 200)
		{
			CountStep += 1;
			LastStepTime = CurrTime;
			HighAcc = 0;
		}
	}
}

void draw()
{
	switch (FsmState)
	{
	case CLOCK_STATE:
		drawClock();
		break;
	case COMPASS_STATE:
		drawCompass();
		break;
	case NAVIGATION_STATE:
		drawNav();
		break;
	}
}

void drawClock()
{
	u8g2.setFont(u8g2_font_logisoso28_tr);
	u8g2.setCursor(0, 46);
	u8g2.print(CurrTime);
	u8g2.setFont(u8g2_font_ncenB12_tf);
	u8g2.setCursor(20, 15);
	u8g2.print(CurrDate);
	u8g2.setFont(u8g2_font_ncenB10_tf);
	//u8g2.setFontDirection(0);
	String CountStr = String(CountStep);
	u8g2.drawStr(39 - 4 * CountStr.length(), 62, "Steps:");
	u8g2.drawStr(88 - 4 * CountStr.length(), 62, CountStr.c_str());
}

void drawNav()
{
	//draw the compass circle
	u8g2.drawCircle(64, 32, 31);
	//if Data is invalid
	if (-1 == Distance)
	{
		u8g2.setFont(u8g2_font_ncenB12_tf);
		//u8g2.setFontDirection(0);
		u8g2.drawStr(17, 34, "Searching...");
	}
	//if Data is valid
	else if (Distance <= 10)
	{
		u8g2.setFont(u8g2_font_ncenB12_tf);
		//u8g2.setFontDirection(0);
		u8g2.drawStr(25, 36, "LED is on");
	}
	else
	{
		//pointing to the destination
		u8g2.drawTriangle(64 + 30 * sin(Course - Angle), 32 - 30 * cos(Course - Angle),//The coordinate of one vertex
			64 + 20 / cos(0.281) * sin(Course - Angle - 0.281), 32 - 20 / cos(0.281) * cos(Course - Angle - 0.281),//The coordinate of another vertex
			64 + 20 / cos(0.281) * sin(Course - Angle + 0.281), 32 - 20 / cos(0.281) * cos(Course - Angle + 0.281));//The coordinate of the last vertex
		u8g2.setFont(u8g2_font_ncenB10_tf);
		u8g2.setFontDirection(0);
		String Dis;
		Dis = String((int)Distance) + 'm';
		//draw the distance to the destination
		u8g2.drawStr(62 - 4 * Dis.length(), 36, Dis.c_str());
	}
	//if the data is invalid
	/*else
	{
		u8g2.setFont(u8g2_font_ncenB12_tf);
		u8g2.setFontDirection(0);
		u8g2.drawStr(17, 34, "Searching...");
	}*/
}

void drawCompass()
{
	//draw the compass circle
	u8g2.drawCircle(64, 32, 26);
	u8g2.setFont(u8g2_font_ncenB10_tf);
	//u8g2.setFontDirection(0);
	//draw the location of North
	u8g2.drawStr(58 - 26 * sin(Angle), 38 - 26 * cos(Angle), "N");
	//pointing to the front
	u8g2.drawTriangle(64, 2,//The coordinate of one vertex
		64 - 20 * sin(-0.281), 32 - 20 * cos(-0.281),//The coordinate of another vertex
		64 - 20 * sin(0.281), 32 - 20 * cos(0.281));//The coordinate of the last vertex
	//u8g2.drawStr(44, 47, String(degrees(Angle)).c_str());
}