#ifndef IO_H
#define IO_H

void ICACHE_FLASH_ATTR ioLed(int ena);
void ioInit(void);
//void ICACHE_FLASH_ATTR handleUpgrade(uint8_t serverVersion, const char *server_ip, uint16_t port, const char *path);
//void ICACHE_FLASH_ATTR ota_finished_callback(void *arg);
#endif
