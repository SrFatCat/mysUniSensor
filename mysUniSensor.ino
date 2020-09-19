/*
 Name:		mysUniSensor.ino
 Created:	06.02.2020 
 Edited: 	06.05.2020
 Author:	Alexey Bogdan aka Sr.FatCat

Первая версия моей платки, пока только датчик температурки, датчик светика и немного движухи
Кроме того, отладим энергопотребление и сон
*/

#define MY_PROJECT_NAME "Uni sensor"
#define MY_PROJECT_VERSION "1.3"

#define MY_DEBUG
#ifndef MY_DEBUG
#define MY_DISABLED_SERIAL
#endif

#define MY_NODE_ID 9
#define MY_PARENT_NODE_ID 50
#define MY_RADIO_NRF5_ESB

int16_t myTransportComlpeteMS;
#define MY_TRANSPORT_WAIT_READY_MS (myTransportComlpeteMS)
#define MY_SEND_RESET_REASON 252
#define MY_SEND_BATTERY 253
//#define MY_SEND_RSSI 254


//#define LIGHT_SENSOR_BH // определен - BH1750FVI, нет - MAX44009
#include <DallasTemperature.h>
#include <OneWire.h>
#include <MySensors.h>
#include <efektaGpiot.h>
#include <strongNode.h>
#include <Wire.h>
#ifdef LIGHT_SENSOR_BH
#include <BH1750FVI.h>
#else
#include <Max44009.h>
#endif

#define INTERVAL_MEASUREMENT 20000//(15*60000UL) //15 min
//#define INTERVAL_PIR_SEND (2 * 60000UL)//(15*60000UL) //15 min
#define INTERVAL_WDT 100000

#ifdef LIGHT_SENSOR_BH
BH1750FVI LightSensor(BH1750FVI::k_DevModeOneTimeHighRes);
#else
Max44009 LightSensor(0x4A);
#endif

#define ONE_WIRE_BUS 12
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature ds18b20(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

#define PIN_CCS811_WAKE 2
#define PIN_CCS811_RESET 4
#include "Adafruit_CCS811.h"
Adafruit_CCS811 ccs;

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

uint8_t sendMeasurement(bool enableLED = true) {
	static float pTemperature = -127.0;
	static uint16_t pLight = 0xFFFF; 
	int ret = 0;
	bool isSendTemp = false;

	if (enableLED) digitalWrite(LED_G, LOW);
	
	ds18b20.requestTemperatures();
	int16_t conversionTime = ds18b20.millisToWaitForConversion(ds18b20.getResolution());
	wait(50);
	if (enableLED) digitalWrite(LED_G, HIGH);
	if (conversionTime > 50) wait(conversionTime-50);

	float temperature = ds18b20.getTempCByIndex(0);
	if (temperature != -127.0){
	 	CORE_DEBUG("*************** Temperature = %3.1f\n", temperature);
		if (pTemperature != temperature ) {
			if (strongNode.sendMsg(msgTemp.set(temperature, 1))) pTemperature = temperature; else ret++;
			isSendTemp = true;
		}
			
	}
	else {
		CORE_DEBUG("*************** ERR: not get temperature\n");
	}

#ifdef LIGHT_SENSOR_BH
	LightSensor.Reset();
	LightSensor.SetMode(BH1750FVI::k_DevModeOneTimeHighRes);
	wait(500);
	uint16_t light = LightSensor.GetLightIntensity();
	LightSensor.Sleep();
#else
	uint16_t light = LightSensor.getLux();
	if (LightSensor.getError()) {
		CORE_DEBUG("*************** Light sensor ERR = %i LUX\n", LightSensor.getError());
		light = pLight;
	}
#endif

	if (isSendTemp && enableLED) strongNode.takeVoltage();

	CORE_DEBUG("*************** Light intensity = %i LUX\n", light);
	if (light != pLight) {
		if(strongNode.sendMsg(msgLight.set(light))) pLight = light; else ret++;
		if (!isSendTemp && enableLED) strongNode.takeVoltage();
	}

	//strongNode.sendSignalStrength();
	strongNode.sendBattery();

	if (enableLED) errLed(ret);

	return ret;
}

void before(){
	wdt_enable(INTERVAL_WDT);
	for (int i = 0; i < 3; i++) {
		hwPinMode(leds[i], OUTPUT);
		digitalWrite(leds[i], LOW);
	}
	hwPinMode(PIR_PIN, INPUT);
	hwPinMode(PIN_CCS811_WAKE, OUTPUT);
	hwPinMode(PIN_CCS811_RESET, OUTPUT);
	strongNode.before();

	NRF_NFCT->TASKS_DISABLE = 1;
	NRF_NVMC->CONFIG = 1;
	NRF_UICR->NFCPINS = 0;
	NRF_NVMC->CONFIG = 0;
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

	digitalWrite(PIN_CCS811_WAKE, LOW);
	digitalWrite(PIN_CCS811_RESET, LOW);
	wait(100);
	digitalWrite(PIN_CCS811_RESET, HIGH);
	
	wait(1000);
	if(!ccs.begin())
		CORE_DEBUG("Failed to start CCS811! Please check your wiring.\n");
	else CORE_DEBUG("Start CCS811! *********************\n");		
	while(!ccs.available()) {
		CORE_DEBUG("Wait to CCS811 available...\n");
		wait(1000);
	}
	float temp = ccs.calculateTemperature();
	CORE_DEBUG("CCS811 calculateTemperature: %f\n", temp);
	ccs.setTempOffset(temp - 25.0);
	while(1) {
		while(!ccs.available()) { CORE_DEBUG("CCS811 available ERROR!\n"); wait(1000); }
		CORE_DEBUG("CCS811 ok!\n");
		float temp = ccs.calculateTemperature();
		uint8_t err = ccs.readData();
		if (!err){
			CORE_DEBUG("CCS811 CO2: %ippm TVOC: %ippb, %foС\n", ccs.geteCO2(), ccs.getTVOC(), temp);
		}
		else {
			CORE_DEBUG("CCS811 read ERROR %i ...init\n",err);
			digitalWrite(PIN_CCS811_RESET, LOW);
			wait(100);
			digitalWrite(PIN_CCS811_RESET, HIGH);
			ccs.begin();
		}			
	 	wait(2000);
	}


	interruptedSleep.addPin(PIR_PIN, NRF_GPIO_PIN_PULLDOWN, CDream::NRF_PIN_LOW_TO_HIGH); // добавляем описание пинов
	interruptedSleep.init();
	
	strongNode.setup();
	
	NRF_POWER->DCDCEN = 1;

	ds18b20.begin();
	ds18b20.setResolution(12);
	ds18b20.setWaitForConversion(false);

#ifdef LIGHT_SENSOR_BH
	LightSensor.begin();
#else
	Wire.begin();
	LightSensor.setAutomaticMode();
	//LightSensor.setContinuousMode();
#endif	
	sendMeasurement();
	CORE_DEBUG("*************** SQ: %i%%\n", strongNode.getSignalQuality());
	//powerManagment();
}

void loop() {
	static bool isSendMotionOFF = true;
	static uint32_t measuremetInterval = INTERVAL_MEASUREMENT;
	static uint32_t pirSendTime = 0;

	const uint32_t tSleepStart = millis();
	const int8_t wakeupReson = interruptedSleep.run(INTERVAL_MEASUREMENT, INTERVAL_WDT, false);
	if (wakeupReson != MY_WAKE_UP_BY_TIMER){
		CORE_DEBUG("*************** WAKE_UP_BY %i\n", wakeupReson);
		digitalWrite(LED_B, LOW);
		wait(50);
		digitalWrite(LED_B, HIGH);	
		bool sendOk = strongNode.sendMsg(msgMotion.set(true), 5);
		if (!sendOk){
			CORE_DEBUG("*************** try STRONG send\n", wakeupReson);
			NRF_POWER->DCDCEN = 0;
			wait(10);
			sendOk = strongNode.sendMsg(msgMotion.set(true), 5);
			NRF_POWER->DCDCEN = 1;
		}
		errLed(!sendOk);
		if (sendOk)	interruptedSleep.disableInterrupt();
	}
	const uint32_t t = millis();
	if (t - tSleepStart > INTERVAL_MEASUREMENT / 2 /*|| wakeupReson == MY_WAKE_UP_BY_TIMER*/){
		sendMeasurement(wakeupReson == MY_WAKE_UP_BY_TIMER);
	}
	if (wakeupReson == MY_WAKE_UP_BY_TIMER){
		 CORE_DEBUG("*************** WAKE_UP_BY_TIMER\n");
		interruptedSleep.enableInterrupt();
	}


	// const int8_t wakeupReson = interruptedSleep.run(measuremetInterval > INTERVAL_MEASUREMENT ? measuremetInterval = INTERVAL_MEASUREMENT : measuremetInterval, INTERVAL_WDT, false);
	// if (wakeupReson == MY_WAKE_UP_BY_TIMER){
    //     CORE_DEBUG("*************** WAKE_UP_BY_TIMER\n");
	// 	sendMeasurement();
	// 	measuremetInterval = INTERVAL_MEASUREMENT;
	// 	interruptedSleep.enableInterrupt();
    // }
	// else {
	// 	const uint32_t t = millis();
	// 	measuremetInterval -= (t - tSleepStart); // скока времени еще спать до измерения
	// 	//if (measuremetInterval > INTERVAL_MEASUREMENT)  measuremetInterval = INTERVAL_MEASUREMENT; // защита от переполнения беззнакового
	// 	//if (pirSendTime == 0 || t - pirSendTime > INTERVAL_PIR_SEND){ // если проснулись первый раз или после остывания
	// 	digitalWrite(LED_B, LOW);
	// 	wait(50);
	// 	digitalWrite(LED_B, HIGH);	
	// 	bool sendOk = strongNode.sendMsg(msgMotion.set(true), 5);
	// 	errLed(!sendOk);
	// 	if (sendOk){ 
	// 		pirSendTime = t;
	// 		interruptedSleep.disableInterrupt();
	// 	}
	// 	//}
	// 	CORE_DEBUG("*************** WAKE_UP_BY %i, time remain %i of %i \n", wakeupReson,  measuremetInterval, INTERVAL_MEASUREMENT);
	// }
}

void receive(const MyMessage & message){
    if (strongNode.checkAck(message)) return;
}

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
		Serial.print("+");
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

/*
прежде всего модуль должен соответствовать схеме dc-dc, например для модулей от ебайта 52832 нужно допаивпть индуктивности, есть только выводы 
на пады, на модулях от скайлаб 52840 есть места под индуктивности на модуле, там надо их туда допаивпть. а дальше все просто:
NRF_POWER->DCDCEN = 1;

причем иногда я его выключаю для презентации(NRF_POWER->DCDCEN = 0;) а потом опять включаю
*/

/*

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
