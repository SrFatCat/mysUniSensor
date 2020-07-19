
//#define MY_DEBUG
#ifndef MY_DEBUG
#define MY_DISABLED_SERIAL
#endif

#define MY_NODE_ID 11
#define MY_RADIO_NRF5_ESB


#include <MySensors.h>
#include <efektaGpiot.h>

#define PIR_PIN 8
const uint8_t leds[] = {LED_R, LED_G, LED_B};

CDream interruptedSleep(1); // количество пинов по которым будут прерывания сна

void before(){
	//wdt_enable(INTERVAL_WDT);
	for (int i = 0; i < 3; i++) {
		hwPinMode(leds[i], OUTPUT);
		digitalWrite(leds[i], LOW);
	}
	hwPinMode(PIR_PIN, INPUT);
}

void setup() {
	for (int i = 0; i < 3; i++) {
		digitalWrite(leds[i], HIGH);
	}
	wait(2000);
	
	interruptedSleep.addPin(PIR_PIN, NRF_GPIO_PIN_NOPULL, CDream::NRF_PIN_LOW_TO_HIGH); // добавляем описание пинов
	interruptedSleep.init();
}

void loop() {
	int8_t wakeupReson = interruptedSleep.run(5000, 1000, false);
	if (wakeupReson == MY_WAKE_UP_BY_TIMER){
		digitalWrite(LED_G, LOW);
		wait(50);
		digitalWrite(LED_G, HIGH);
    }
	else {
		digitalWrite(LED_B, LOW);
		wait(50);
		digitalWrite(LED_B, HIGH);
    }
}

void receive(const MyMessage & message){
//    if (strongNode.checkAck(message)) return;
}
