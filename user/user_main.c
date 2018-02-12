/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Jeroen Domburg <jeroen@spritesmods.com> wrote this file. As long as you retain
 * this notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer in return.
 * ----------------------------------------------------------------------------
 */

/*
This is example code for the esphttpd library. It's a small-ish demo showing off
the server, including WiFi connection management capabilities, some IO and
some pictures of cats.
*/

#include <esp8266.h>
#include "httpd.h"
#include "io.h"
#include "httpdespfs.h"
#include "cgi.h"
#include "cgiwifi.h"
#include "cgiflash.h"
#include "stdout.h"
#include "auth.h"
#include "mqtt.h"
#include "espfs.h"
#include "captdns.h"
#include "webpages-espfs.h"
#include "config.h"
#include "gpio16.h"
#include "dht22.h"
#include "bridgethings.h"
#include "wifi.h"
#include "ultrasonic.h"

MQTT_Client mqttClient;
static ETSTimer WiFiLinker;
typedef void (*WifiCallback)(uint8_t);
os_timer_t duty_timer,calTimer,readingsTimer,discovery_timer;
WifiCallback wifiCb = NULL;
static uint8_t wifiStatus = STATION_IDLE, lastWifiStatus = STATION_IDLE;
uint8_t disFlag=0,intlComplete=0;
char macID[16];
float calGas = 0.0f;
ultrasonic_sensor_t sensor;
sensor_info sinfo[MAX_SENSORS];
uint8_t totalSensors = 0;
char discoveryPacket[25];
char configChannel[50];
void ICACHE_FLASH_ATTR wifi_check_ip(void *arg)
{
	struct ip_info ipConfig;

	os_timer_disarm(&WiFiLinker);
	wifi_get_ip_info(STATION_IF, &ipConfig);
	wifiStatus = wifi_station_get_connect_status();
	if (wifiStatus == STATION_GOT_IP && ipConfig.ip.addr != 0)
	{
		os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
		os_timer_arm(&WiFiLinker, 2000, 0);
	}
	else
	{
		if(wifi_station_get_connect_status() == STATION_WRONG_PASSWORD)
		{
			INFO("STATION_WRONG_PASSWORD\r\n");
			wifi_station_connect();
		}
		else if(wifi_station_get_connect_status() == STATION_NO_AP_FOUND)
		{
			INFO("STATION_NO_AP_FOUND\r\n");
			wifi_station_connect();
		}
		else if(wifi_station_get_connect_status() == STATION_CONNECT_FAIL)
		{
			INFO("STATION_CONNECT_FAIL\r\n");
			wifi_station_connect();
		}
		else
		{
			INFO("STATION_IDLE\r\n");
		}

		os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
		os_timer_arm(&WiFiLinker, 500, 0);
	}
	if(wifiStatus != lastWifiStatus){
		lastWifiStatus = wifiStatus;
		if(wifiCb)
		{
			wifiCb(wifiStatus);
		}

	}
}
void ICACHE_FLASH_ATTR WIFI_Connect(uint8_t* ssid, uint8_t* pass, WifiCallback cb){

	struct station_config stationConf;
	wifi_set_opmode_current(STATION_MODE);
	wifi_station_set_auto_connect(FALSE);
	wifiCb = cb;

	os_memset(&stationConf, 0, sizeof(struct station_config));

	os_sprintf((char*)stationConf.ssid, "%s", ssid);
	os_sprintf((char*)stationConf.password, "%s", pass);

	wifi_station_set_config_current(&stationConf);

	//Check wifi status and try to reconnect.
	os_timer_disarm(&WiFiLinker);
	os_timer_setfn(&WiFiLinker, (os_timer_func_t *)wifi_check_ip, NULL);
	os_timer_arm(&WiFiLinker, 5000, 0); // 1 second timer for wifi.

	wifi_station_set_auto_connect(TRUE);
	wifi_station_connect();
}
void ICACHE_FLASH_ATTR wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		uint8 macaddr[6];
		wifi_get_macaddr(0x00,macaddr); // wifi should be connected for this to work
		os_sprintf(macID, MACSTRING,MAC2STRING(macaddr));
		MQTT_InitLWT(&mqttClient,(uint8_t*)lastWill, (uint8_t*)macID, 2, 1);
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}

void ICACHE_FLASH_ATTR motion_detected(unsigned pin, unsigned level)
{
	char motionMsg[20];
	strcat(motionMsg,macID);

	if(level == 1){
	INFO("Motion detected\n");
	strcat(motionMsg,":1");
	}
	else{
	INFO("Motion stopped \n");
	strcat(motionMsg,":0");
	}
	MQTT_Publish(&mqttClient,motionChannel,motionMsg,sizeof(motionMsg),2,0);
}
LOCAL void ICACHE_FLASH_ATTR dht22_cb()
{

	struct dht_sensor_data* r;
	static char temp[10] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
	char *tempHumMsg;
	static char hum[10]={NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
	float lastTemp, lastHum;

	r = DHTRead();
	lastTemp = r->temperature;
	lastHum = r->humidity;
	INFO("last temp: %d",(int)lastTemp);
	INFO("last hum: %d",(int)lastHum);
	os_sprintf(temp, "%d.%d",(int)(lastTemp),(int)((lastTemp - (int)lastTemp)*100));
	os_sprintf(hum, "%d.%d",(int)(lastHum),(int)((lastHum - (int)lastHum)*100));
	os_printf("Temperature: %s *C, Humidity: %s %%\r\n", temp, hum);
	tempHumMsg = (char*)os_zalloc(strlen(temp)+strlen(hum)+strlen(macID)+3);
	strcat(tempHumMsg,macID);
	strcat(tempHumMsg,":");
	strcat(tempHumMsg,temp);
	strcat(tempHumMsg,":");
	strcat(tempHumMsg,hum);
	MQTT_Publish(&mqttClient,tempHumChannel,tempHumMsg,strlen(tempHumMsg),2,0);
    os_free(tempHumMsg);
}

void ICACHE_FLASH_ATTR initializeMotionSensor()
{
	INFO("Initializing motion sensor \n");
	set_gpio_mode(2,GPIO_INT,GPIO_PULLDOWN);
	gpio_intr_init(2, GPIO_PIN_INTR_ANYEDGE);
	gpio_intr_attach(motion_detected);
}
void ICACHE_FLASH_ATTR discoveryCallback(void *arg)
{
  INFO("Sending discovery packet \r\n");
  MQTT_Publish(&mqttClient,discoveryChannel,discoveryPacket,strlen(discoveryPacket),0,0);

}
void ICACHE_FLASH_ATTR calibrationCb(void *args)
{
     static float calres = 0;
	 static int i=0;
	 caldata* calibrationData  = (caldata*)args;
	 int samples = calibrationData->samples;
	 INFO("Arguments %d\n",samples);
	// INFO("Arguments %d\n",cal);
	 int adc_value=system_adc_read();
	 INFO("ADC value: %d \n",adc_value);
	 calres +=((float)load_Res*(1023-adc_value)/adc_value);
	 INFO("ADC value: %d \n",adc_value);
	 i++;
	 if(i == samples)
	 {
		 INFO("Cal res %d",(int)calres);
		 calres/=samples;
		 INFO("Cal res %d",(int)calres);
		 calGas = (float)((float)calres/(float)air_factor);
		 INFO("Cal gas %d",(int)calGas);
		 intlComplete=1;
		 INFO("Ini complete \n");
		 i=0;
		 calres = 0.0f;
		 os_timer_disarm(&calTimer);
		 os_free(calibrationData);
	 }

}
void ICACHE_FLASH_ATTR gasReadingsCb(void *args)
{
     static float res=0.0f;
	 static int i=0;
	 caldata* calibrationData  = (caldata*)args;
	 int samples = calibrationData->samples;
	 INFO("Arguments %d\n",samples);
	// INFO("Arguments %d\n",cal);
	 int adc_value=system_adc_read();
	 INFO("ADC value: %d \n",adc_value);
	 res +=((float)load_Res*(1023-adc_value)/adc_value);
	 i++;
	 if(i==samples)
	 {
		os_timer_disarm(&readingsTimer);
		res/=calGas;  //(Rs/Ro) point y2
		INFO("Res value: %d", (int)res);
		char *gasMsg;
		char gasRes[10] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
		os_sprintf(gasRes, "%d.%d",(int)(res),(int)((res - (int)res)*100));
		int len = strlen(macID)+strlen(gasRes) + 2;
		gasMsg = (char *)os_zalloc(len);
		strcat(gasMsg,macID);
		strcat(gasMsg,":");
		strcat(gasMsg,gasRes);
		INFO("Sending gas data %s", gasMsg);
		MQTT_Publish(&mqttClient,gasChannel,gasMsg,strlen(gasMsg),2,0);
		i=0;
		res = 0.0f;
        os_free(calibrationData);
        os_free(gasMsg);
	 }

}

void ICACHE_FLASH_ATTR initializeSensors()
{

	    int j=0;
		INFO("Initializing sensors \n");
		for(j=0;j<totalSensors;j++)
		{

			switch(sinfo[j].senseID)
			{

				case DHT11_TEMP:
					DHTInit(DHT11);
					break;
				case DHT22_TEMP:
					DHTInit(DHT22);
					break;
				case PIR:
					initializeMotionSensor();
					break;
				case MICROWAVE_MOTION:
					initializeMotionSensor();
					break;
				case MQ2:
					intlComplete=0;
					caldata *calibrationData = (caldata*)os_zalloc(sizeof(caldata));
					calibrationData->samples = 50;
					calibrationData->interval = 100;
					os_timer_disarm(&calTimer);
					os_timer_setfn(&calTimer,(os_timer_func_t *)calibrationCb,calibrationData);
					os_timer_arm(&calTimer,calibrationData->interval,1);
					break;
				case HRSC04:
				{
					sensor.trigger_pin = TRIGGER_PIN;
					sensor.echo_pin = ECHO_PIN;
					ultrasoinc_init(&sensor);
					break;
				}
				default:
					break;

			}

		}


}


void ICACHE_FLASH_ATTR getMQ2Readings()
{
	if(intlComplete)
	{
		//INFO("Starting timer for mq2");
		caldata *calibrationData = (caldata*)os_zalloc(sizeof(caldata));
		calibrationData->samples = 10;
		calibrationData->interval = 500;
		os_timer_disarm(&readingsTimer);
		os_timer_setfn(&readingsTimer,(os_timer_func_t *)gasReadingsCb,calibrationData);
		os_timer_arm(&readingsTimer,calibrationData->interval,1);

	}
}
void ICACHE_FLASH_ATTR sendSensorData(int sensorID, int dutyCycle)
{

	    INFO("Sensor ID %d %d",sensorID,dutyCycle);

		switch(sensorID)
		{

			case DHT11_TEMP:
			{
				 static int i=0;
				 i++;
				 if(i == dutyCycle/30)
				 {
					 // GET SENSOR readings here
					 INFO("sending DHT11 data \n");
					 dht22_cb();
					 i=0;
				 }
				 break;
			}
			case DHT22_TEMP:
			{	static int i =0;
				i++;
				if(i == dutyCycle/30)
				{
					//get sensor readings here
					INFO("sending DHT22 data \n");
					i=0;
				}
				 break;
			}
			case HRSC04:
			{
				static int i =0;
				i++;
				if(i == dutyCycle/30)
				{
					INFO("Sending ultrasound data \n");
					int32_t distance = ultrasoinc_measure_cm(&sensor, 500);
                    char dis[10];
				    os_sprintf(dis,"%d",distance);
				    char *distanceMsg = (char*)os_zalloc(strlen(macID)+strlen(dis)+2);
				    strcat(distanceMsg,macID);
				    strcat(distanceMsg,":");
				    strcat(distanceMsg,dis);
				    INFO("Distance %s \n\n",distanceMsg);
				    MQTT_Publish(&mqttClient,ultraChannel,distanceMsg,strlen(distanceMsg),2,0);
				    os_free(dis);
				    os_free(distanceMsg);
					i=0;
				}
				break;
			}
			case MQ2:
			{
				static int i =0;
				i++;
				if(i == dutyCycle/30)
				{
					INFO("Sending MQ2 readings \n");
					getMQ2Readings();
					i=0;
				}
				break;
			}
			case TSL14SM:
			{
				static int i =0;
				i++;
				if(i == dutyCycle/30)
				{
					INFO("Getting ambient light readings \n");
					int light_value = system_adc_read();
					char light[5];
					os_sprintf(light,"%d",light_value);
					char *lightMsg = (char*)os_zalloc(strlen(macID)+strlen(light)+2);
					strcat(lightMsg,macID);
					strcat(lightMsg,":");
					strcat(lightMsg,light);
					MQTT_Publish(&mqttClient,ldrChannel,lightMsg,strlen(lightMsg),2,0);
					os_free(lightMsg);
					i=0;
				}
				break;
			}

			default:
				 INFO("In defualt \n");
				 break;
	  }
}

void ICACHE_FLASH_ATTR sensorDuty()
{

	int j=0;
	INFO("Executing sensor duty \n");
	for(j=0;j<totalSensors;j++)
	{
		sendSensorData(sinfo[j].senseID,sinfo[j].duty);
	}

}
void ICACHE_FLASH_ATTR mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");

	if(!disFlag)
	{
		disFlag = 1;
		set_gpio_mode(RELAY_GPIO,GPIO_OUTPUT,GPIO_PULLDOWN);
		gpio_write(RELAY_GPIO,0);
		INFO("mac : %s \n",macID);
		float currentversion = 1.0;
		strcat(discoveryPacket,macID);
		strcat(discoveryPacket,":");
		char currentVer[10];
		os_sprintf(currentVer, "%d.%d",(int)(currentversion),(int)((currentversion - (int)currentversion)*100));
		strcat(discoveryPacket,currentVer);
		strcat(configChannel,baseChannel);
		strcat(configChannel,macID);
	}

	INFO("subscribing to config channel\r\n");
	MQTT_Subscribe(client,configChannel,2);
	os_timer_disarm(&discovery_timer);
	os_timer_setfn(&discovery_timer,(os_timer_func_t *)discoveryCallback,(void *)0);
	os_timer_arm(&discovery_timer,DISCOVERY_DUTY,1);

}

void ICACHE_FLASH_ATTR mqttDisconnectedCb(uint32_t *args)
{
	//MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
	os_timer_disarm(&discovery_timer);
}

void ICACHE_FLASH_ATTR mqttPublishedCb(uint32_t *args)
{
	//MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void ICACHE_FLASH_ATTR mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf = (char*)os_zalloc(topic_len+1),
			*dataBuf = (char*)os_zalloc(data_len+1);

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
	if(!strcmp(topicBuf,configChannel))
	{
	   INFO("Recvd data on config channel \n");
       INFO("Data rcvd: %s",dataBuf);
       char *configCommand  = dataBuf;
       char *actualCommand = strsep(&configCommand,":");
       switch(actualCommand[0])
       {

		   case 'a':
			   INFO("Rcvd ack for discovery\r\n");
			   os_timer_disarm(&discovery_timer);
			   break;
		   case 'r':
			   INFO("Recvd a relay command\n");
			   long int relayState = strtol(configCommand,NULL,10);
			   INFO("Rcvd relay state: %ld",relayState);
			   if(relayState == 0)
				   gpio_write(RELAY_GPIO,0);
			   else
				   gpio_write(RELAY_GPIO,1);
			   break;
		   case 'c':
		   {
			   char* found;
			   INFO("Recvd a config command \n");
			   totalSensors=0;
			   while( (found = strsep(&configCommand,",")) != NULL )
			   {
                    sinfo[totalSensors].senseID = (int)strtol(strsep(&found,"/"),NULL,10);
                    sinfo[totalSensors].duty = (int)strtol(found,NULL,10);
                    INFO("Recvd senseID & duty %d %d \n",sinfo[totalSensors].senseID,sinfo[totalSensors].duty);
                    totalSensors++;;
			   }
			   initializeSensors();
			   os_timer_disarm(&duty_timer);
			   os_timer_setfn(&duty_timer,(os_timer_func_t *)sensorDuty, NULL);
			   os_timer_arm(&duty_timer,30000,1);
			   break;
		   }
		   /*case 'u': for OTA update
			   INFO("Recvd a relay command\n");
			   float update = atof(configCommand);
			   INFO("Rcvd relay state: %f",update);
			   handleUpgrade(2,"asdf",45,"asdf");
			   break;*/
		   default:
			   break;
	}
	}
	os_free(topicBuf);
	os_free(dataBuf);
}

int ICACHE_FLASH_ATTR myPassFn(HttpdConnData *connData, int no, char *user, int userLen, char *pass, int passLen) {
	if (no==0) {
		os_strcpy(user, "admin");
		os_strcpy(pass, "s3cr3t");
		return 1;
	}
	return 0;
}

#ifdef ESPFS_POS
CgiUploadFlashDef uploadParams={
	.type=CGIFLASH_TYPE_ESPFS,
	.fw1Pos=ESPFS_POS,
	.fw2Pos=0,
	.fwSize=ESPFS_SIZE,
};
#define INCLUDE_FLASH_FNS
#endif
#ifdef OTA_FLASH_SIZE_K
CgiUploadFlashDef uploadParams={
	.type=CGIFLASH_TYPE_FW,
	.fw1Pos=0x1000,
	.fw2Pos=((OTA_FLASH_SIZE_K*1024)/2)+0x1000,
	.fwSize=((OTA_FLASH_SIZE_K*1024)/2)-0x1000,
	.tagName= "BRIDGE" //OTA_TAGNAME
};
#define INCLUDE_FLASH_FNS
#endif

HttpdBuiltInUrl builtInUrls[]={
	{"*", cgiRedirectApClientToHostname, "bt.iot"},
	{"/", cgiRedirect, "wifi/wifi.tpl"},
	{"/wifi", cgiRedirect, "/wifi/wifi.tpl"},
	{"/wifi/", cgiRedirect, "/wifi/wifi.tpl"},
	{"/wifi/wifiscan.cgi", cgiWiFiScan, NULL},
	{"/wifi/wifi.tpl", cgiEspFsTemplate, tplWlan},
	{"/wifi/connect.cgi", cgiWiFiConnect, NULL},
	{"/wifi/connstatus.cgi", cgiWiFiConnStatus, NULL},
	{"/wifi/setmode.cgi", cgiWiFiSetMode, NULL},
	{"*", cgiEspFsHook, NULL}, //Catch-all cgi function for the filesystem
	{NULL, NULL, NULL}
};


uint32 ICACHE_FLASH_ATTR user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 8;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void ICACHE_FLASH_ATTR initDone_cb()
{

	wifiCb = wifiConnectCb;
	INFO("Mqtt host: %s \n", sysCfg.mqtt_host);
	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);
	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

}

//Main routine. Initialize stdout, the I/O, filesystem and the webserver and we're done.
void ICACHE_FLASH_ATTR user_init(void)
{
	stdoutInit();
	ioInit();
	CFG_Load();
	char configMode[2];
	os_sprintf(configMode,"%s",sysCfg.configMode);
	os_printf("Config Mode: %s\n",configMode);

if(!strcmp(configMode,"1"))
{
	//wifi_set_opmode(0x03);
	wifiInit();
	INFO("Booting in config mode\n");
	captdnsInit();
	// 0x40200000 is the base address for spi flash memory mapping, ESPFS_POS is the position
	// where image is written in flash that is defined in Makefile.
#ifdef ESPFS_POS
	espFsInit((void*)(0x40200000 + ESPFS_POS));
#else
	espFsInit((void*)(webpages_espfs_start));
#endif
	httpdInit(builtInUrls, 80);

	/*os_timer_disarm(&websockTimer);
	os_timer_setfn(&websockTimer, websockTimerCb, NULL);
	os_timer_arm(&websockTimer, 1000, 1);*/
}
else
{
	/*Add by Raju Namburi */
	INFO("Booting in working mode \n");
	system_init_done_cb(&initDone_cb);
}
}
