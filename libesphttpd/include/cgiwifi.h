#ifndef CGIWIFI_H
#define CGIWIFI_H

#include "httpd.h"
#ifndef MAC2STRING
#define MAC2STRING(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTRING "%02x%02x%02x%02x%02x%02x"
#endif
int cgiWiFiScan(HttpdConnData *connData);
int tplWlan(HttpdConnData *connData, char *token, void **arg);
int cgiWiFi(HttpdConnData *connData);
int cgiWiFiConnect(HttpdConnData *connData);
int cgiWiFiSetMode(HttpdConnData *connData);
int cgiWiFiConnStatus(HttpdConnData *connData);

#endif
