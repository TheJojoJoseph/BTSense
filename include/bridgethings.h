#ifndef __BRIDGE_THINGS_H__
#define __BRIDGE_THINGS_H__

typedef struct sensor_info
{
	int senseID;
	int duty;

}sensor_info;

typedef struct caldata
 {
	  int samples;
	  int interval;
 }caldata;
#define RESET_BUTTON 3
#ifndef MAC2STRING
#define MAC2STRING(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTRING "%02x%02x%02x%02x%02x%02x"
#endif
#define TRIGGER_PIN 6
#define ECHO_PIN 5
#define RELAY_GPIO 7
#define MOTION_SENSOR 6
#define discoveryChannel  "/bridgethings/discovery"
#define baseChannel "/bridgethings/"
#define motionChannel  "/bridgethings/sensor/motion"
#define tempHumChannel  "/bridgethings/sensor/temp_hum"
#define ldrChannel  "/bridgethings/sensor/ldr"
#define soilChannel  "/bridgethings/sensor/soil_moisture"
#define ultraChannel  "/bridgethings/sensor/ultra_sound"
#define rtcChannel  "/bridgethings/sensor/rtc_clock"
#define gasChannel  "/bridgethings/sensor/gas"
#define lastWill "/bridgethings/lastwill"

#define sensor A0
#define b 1 // load resistance in series with the sensor heater
#define air_factor 9.83  // RS in fresh air

#define MAX_SENSORS 3
#define DISCOVERY_DUTY 5000

//Temp Hum sensors
#define DHT11_TEMP 120
#define DHT22_TEMP 121


// Motion Sensors
#define PIR 170
#define MICROWAVE_MOTION 171

//Pollution sensors
#define MQ2 220
#define MQ7 221
#define MQ135 222

#define HRSC04 270
#define DYPME007 271

//Temp sensors
#define LM35 320

//LDR
#define TSL14SM 370

#endif
