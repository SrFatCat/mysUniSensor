/*
 Name:		mysUniSensor.ino
 Created:	06.02.2020 
 Edited: 	06.05.2020
 Author:	Alexey Bogdan aka Sr.FatCat

Первая версия моей платки, пока только датчик температурки, датчик светика и немного движухи
Кроме того, отладим энергопотребление и сон
*/

#define MY_PROJECT_NAME "Uni sensor"
#define MY_PROJECT_VERSION "1.1"

//#define MY_DEBUG
#ifndef MY_DEBUG
#define MY_DISABLED_SERIAL
#endif

#define MY_NODE_ID 11
#define MY_PARENT_NODE_ID 50
#define MY_RADIO_NRF5_ESB

int16_t myTransportComlpeteMS;
#define MY_TRANSPORT_WAIT_READY_MS (myTransportComlpeteMS)
#define MY_SEND_RESET_REASON 252
#define MY_SEND_BATTERY 253
#define MY_SEND_RSSI 254

#include <DallasTemperature.h>
#include <OneWire.h>
#include <MySensors.h>
#include <efektaGpiot.h>
#include <strongNode.h>
#include <Wire.h>
#include <BH1750FVI.h>

#define INTERVAL_MEASUREMENT 15*60000UL //15 min
#define INTERVAL_PIR_COOLING 30000UL
#define INTERVAL_WDT 100000//(INTERVAL_MEASUREMENT + 2000UL)*32768UL/1000UL

BH1750FVI LightSensor(BH1750FVI::k_DevModeOneTimeHighRes);

#define ONE_WIRE_BUS 12
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature ds18b20(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

#define CHILD_ID_TEMP 1
#define CHILD_ID_MOTION 2
#define CHILD_ID_LIGHT 3

MyMessage 	msgTemp(CHILD_ID_TEMP, V_TEMP),
			msgMotion(CHILD_ID_MOTION, V_TRIPPED),
			msgLight(CHILD_ID_LIGHT, V_LIGHT_LEVEL);


#define PIR_PIN 8
const uint8_t leds[] = {LED_R, LED_G, LED_B};

void strongPresentation(); 
CStrongNode strongNode(100, strongPresentation);
CDream interruptedSleep(1); // количество пинов по которым будут прерывания сна

void powerManagment(){
	CORE_DEBUG("*************** SQ: %i%%\n", strongNode.getSignalQuality());
	if (strongNode.getSignalQuality() > 25)
		NRF_POWER->DCDCEN = 1;
	else
		NRF_POWER->DCDCEN = 0;
}

void errLed(uint8_t errNum){
	if (errNum == 0) return;

	wait(200);
	for (int i = 0; i < errNum; i++){
		digitalWrite(LED_R, LOW);
		wait(70);
		digitalWrite(LED_R, HIGH);
		if (i < errNum-1 ) wait(500);
	}
}

uint8_t sendMeasurement() {
	static float pTemperature = -127.0;
	static uint16_t pLight = 0xFFFF; 
	int ret = 0;

	digitalWrite(LED_G, LOW);
	
	ds18b20.requestTemperatures();
	int16_t conversionTime = ds18b20.millisToWaitForConversion(ds18b20.getResolution());
	wait(50);
	digitalWrite(LED_G, HIGH);
	if (conversionTime > 50) wait(conversionTime-50);

	float temperature = ds18b20.getTempCByIndex(0);
	if (temperature != -127.0){
	 	CORE_DEBUG("*************** Temperature = %3.1f\n", temperature);
		if (pTemperature != temperature ) {
			if (strongNode.sendMsg(msgTemp.set(temperature, 1))) pTemperature = temperature; else ret++;
		}
			
	}
	else {
		CORE_DEBUG("*************** ERR: not get temperature\n");
	}

	LightSensor.Reset();
	LightSensor.SetMode(BH1750FVI::k_DevModeOneTimeHighRes);
	wait(500);
	uint16_t light = LightSensor.GetLightIntensity();
	CORE_DEBUG("*************** Light intensity = %i LUX\n", light);
	if (light != pLight) {
		if(strongNode.sendMsg(msgLight.set(light))) pLight = light; else ret++;
	}
	LightSensor.Sleep();

	strongNode.sendSignalStrength();
	strongNode.sendBattery();

	errLed(ret);

	return ret;
}

void before(){
	wdt_enable(INTERVAL_WDT);
	for (int i = 0; i < 3; i++) {
		hwPinMode(leds[i], OUTPUT);
		digitalWrite(leds[i], LOW);
	}
	hwPinMode(PIR_PIN, INPUT);
	strongNode.before();
}

void strongPresentation() {
	strongNode.sendSketchInform(MY_PROJECT_NAME, MY_PROJECT_VERSION);
	strongNode.perform(CHILD_ID_TEMP, S_TEMP, "Temperature");
	strongNode.perform(CHILD_ID_MOTION, S_MOTION, "Motion");
	strongNode.perform(CHILD_ID_LIGHT, S_LIGHT_LEVEL, "Light level");
}

void setup() {
	for (int i = 0; i < 3; i++) {
		digitalWrite(leds[i], HIGH);
	}
	wait(2000);
	CORE_DEBUG("\n*************** Start NRF52 mysUniSensor *************** \n\n");
	
	interruptedSleep.addPin(PIR_PIN, NRF_GPIO_PIN_NOPULL, CDream::NRF_PIN_LOW_TO_HIGH); // добавляем описание пинов
	interruptedSleep.init();

	strongNode.setup();


	ds18b20.begin();
	ds18b20.setResolution(12);
	ds18b20.setWaitForConversion(false);
	LightSensor.begin();
	
	sendMeasurement();
	CORE_DEBUG("*************** SQ: %i%%\n", strongNode.getSignalQuality());
	powerManagment();
}

void loop() {
	static bool isSendMotionOFF = true;
	static uint32_t measuremetInterval = INTERVAL_MEASUREMENT;

	const uint32_t tSleepStart = millis();
	// const int8_t wakeupReson = interruptedSleep.run(measuremetInterval > INTERVAL_MEASUREMENT ? measuremetInterval = INTERVAL_MEASUREMENT : measuremetInterval);
	// wdt_reset();
	const int8_t wakeupReson = interruptedSleep.run(measuremetInterval > INTERVAL_MEASUREMENT ? measuremetInterval = INTERVAL_MEASUREMENT : measuremetInterval, INTERVAL_WDT, false);
	if (wakeupReson == MY_WAKE_UP_BY_TIMER){
        CORE_DEBUG("*************** WAKE_UP_BY_TIMER\n");
		if (!isSendMotionOFF) isSendMotionOFF = strongNode.sendMsg(msgMotion.set(false), 5); //если раньше не удалось отправить PIR в 0 повторяем
		sendMeasurement();
		powerManagment();
		measuremetInterval = INTERVAL_MEASUREMENT;
    }
	else {
		measuremetInterval -= (millis() - tSleepStart); // скока времени еще спать до измерения
		if (measuremetInterval > INTERVAL_MEASUREMENT)  measuremetInterval = INTERVAL_MEASUREMENT; // защита от переполнения беззнакового
		CORE_DEBUG("*************** WAKE_UP_BY %i, TIMER remain %i of %i\n", wakeupReson, measuremetInterval, INTERVAL_MEASUREMENT);
		digitalWrite(LED_B, LOW);
		wait(50);
		digitalWrite(LED_B, HIGH);
		bool sendOk = strongNode.sendMsg(msgMotion.set(true), 5);
		errLed(!sendOk);
		interruptedSleep.disableInterrupt();
		while(digitalRead(PIR_PIN)) {
			interruptedSleep.run(INTERVAL_PIR_COOLING, INTERVAL_WDT, false);
			// interruptedSleep.run(INTERVAL_PIR_COOLING);
			// wdt_reset();
			if (measuremetInterval <= INTERVAL_PIR_COOLING) {
				sendMeasurement();
				measuremetInterval = INTERVAL_MEASUREMENT;	
			}
			else measuremetInterval -=INTERVAL_PIR_COOLING;
		}			
		if (sendOk) isSendMotionOFF = strongNode.sendMsg(msgMotion.set(false), 5);
		errLed(isSendMotionOFF ? 0 : 2);
		interruptedSleep.enableInterrupt();
	}
}

void receive(const MyMessage & message){
    if (strongNode.checkAck(message)) return;
}

/*
int nDevices;

void i2cScanner() {
	byte error, address;

	CORE_DEBUG("\n***************Scanning"); //Serial.print("___");

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		//Serial.print("+");
		error = Wire.endTransmission();

		if (error == 0)
		{
			CORE_DEBUG("\n***************I2C device found at address 0x%X !\n", address);
			nDevices++;
		}
		else if (error == 4)
		{
			CORE_DEBUG("\n***************Unknown error at address 0x%X\n", address);
		}
		//else Serial.print("-");
	}
	if (nDevices == 0)
		CORE_DEBUG("\n***************No I2C devices found\n\n");
	else
		CORE_DEBUG("\n***************%i device done\n\n", nDevices);
}

прежде всего модуль должен соответствовать схеме dc-dc, например для модулей от ебайта 52832 нужно допаивпть индуктивности, есть только выводы 
на пады, на модулях от скайлаб 52840 есть места под индуктивности на модуле, там надо их туда допаивпть. а дальше все просто:
NRF_POWER->DCDCEN = 1;

причем иногда я его выключаю для презентации(NRF_POWER->DCDCEN = 0;) а потом опять включаю


void startTimer(uint sec){
	//NRF_TIMER1->POWER = 1;
        // Режим таймера 
	NRF_TIMER1->MODE  = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;

//         Получить секунду можно различными вариантами. Например: поделить 
//        тактовый сигнал (16 MHz) на 2 в степени 9 и отсчитать 31250 "тиков" 
	NRF_TIMER1->PRESCALER = 9; 
	NRF_TIMER1->CC[0] = 3907;

        // Активация прерывания 
	NRF_TIMER1->INTENSET = 
       (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);

         // Разрешение прерываний в NVIC и установка приоритета 
	NVIC_SetPriority(TIMER1_IRQn, 15);
	NVIC_EnableIRQ(TIMER1_IRQn);

        // Запуск таймера на счет
	NRF_TIMER1->TASKS_START = 1;
}

extern "C" {
void TIMER1_IRQHandler(void){
	static uint8_t num = 1;
	static bool l= HIGH;
         // Сброс флага, сигнализирующего об окончании счета
	NRF_TIMER1->EVENTS_COMPARE[0] = 0;
	digitalWrite(leds[num], l = !l);
	if (l) num++;
	if (num == 3) num = 1;
        // Очищение счетного регистра
	NRF_TIMER1->TASKS_CLEAR = 1;
}
}


*/