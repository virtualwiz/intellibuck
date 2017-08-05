#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define INF_WAIT	0
#define INF_CC		1
#define INF_CV		2

#define WRN_OLOAD	3
#define WRN_TEMP	4

#define ERR_OLOAD	5
#define ERR_TEMP	6
#define ERR_FAULT	7

#define GPIO_BUZZER 12

#define GPIO_RELAY 13

LiquidCrystal_I2C lcd(0x27, 20, 4);

char serialReceiveBuffer[30];

void setup()
{
  lcd.init();
  lcd.backlight();
  Serial.begin(115200);
  pinMode(GPIO_RELAY,OUTPUT);
  lcd.setCursor(0, 1);
  lcd.print("  OWLET INDUSTRIES  ");
  delay(500);
  tone(GPIO_BUZZER,523,120);delay(150);
  tone(GPIO_BUZZER,659,120);delay(150);
  tone(GPIO_BUZZER,784,120);delay(150);
  tone(GPIO_BUZZER,880,120);delay(150);
  lcd.setCursor(0, 0);
  lcd.print("SET --.--- V -.--- A");
  lcd.setCursor(0, 1);
  lcd.print("OUT --.--- V -.--- A");
  lcd.setCursor(0, 2);
  lcd.print("--- mA -- % ---- mWh");
  lcd.setCursor(0, 3);
  lcd.print("Deep Idle       -- C");
}

const int dataDisplayPositionX[30] = {4,5,7,8,9,13,15,16,17,	4,5,7,8,9,13,15,16,17,	0,1,2,7,8,12,13,14,15,	16,17,0};
const int dataDisplayPositionY[30] = {0,0,0,0,0,0,0,0,0,	1,1,1,1,1,1,1,1,1,	2,2,2,2,2,2,2,2,2,	3,3,3};

void loop()
{
	int dataPtr;
	if(Serial.available())
	{
		delay(20);
		if(Serial.read()=='$')
		{
			for(dataPtr = 0;dataPtr <= 29;dataPtr++)
				serialReceiveBuffer[dataPtr] = Serial.read();
			
			for(dataPtr = 0;dataPtr <= 28;dataPtr++)
			{
				lcd.setCursor(dataDisplayPositionX[dataPtr],dataDisplayPositionY[dataPtr]);
				lcd.write(serialReceiveBuffer[dataPtr]);
			}
			lcd.setCursor(dataDisplayPositionX[29],dataDisplayPositionY[29]);
			
			switch(serialReceiveBuffer[29]-0x30)
			{
				case INF_WAIT:
				lcd.print("Idle            ");digitalWrite(GPIO_RELAY,LOW);break;
				case INF_CV:
				lcd.print("Drive Mode: CV  ");digitalWrite(GPIO_RELAY,HIGH);break;
				case INF_CC:
				lcd.print("Drive Mode: CC  ");digitalWrite(GPIO_RELAY,HIGH);break;
				case WRN_OLOAD:
				lcd.print("Overload        ");break;
				case WRN_TEMP:
				lcd.print("Temp Warning    ");break;				
				case ERR_OLOAD:
				lcd.print("Severe Overload ");digitalWrite(GPIO_RELAY,LOW);break;
				case ERR_TEMP:
				lcd.print("Temp Too High   ");digitalWrite(GPIO_RELAY,LOW);break;
				case ERR_FAULT:
				lcd.print("Hardware Fault  ");digitalWrite(GPIO_RELAY,LOW);break;
				default:
				lcd.print("Commu Error     ");
			}
			
			if(serialReceiveBuffer[29]-0x30 >= 5)
			{
				tone(GPIO_BUZZER,1046,1000);
			}
			else if(serialReceiveBuffer[29]-0x30 >= 3)
			{
				tone(GPIO_BUZZER,1046,10);
			}
						
		}

	}
}
