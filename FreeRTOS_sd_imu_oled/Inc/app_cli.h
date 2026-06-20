/*
 * app_cli.h
 *
 *  UART console — 解析 start / stop / status 命令控制 datalogger。
 *
 *  Created on: Jun 18, 2026
 *      Author: xushu
 */

#ifndef APP_CLI_H_
#define APP_CLI_H_

/* 印出提示字。 */
void app_cli_init(void);

/* 主迴圈每圈呼叫：把 UART RX ring buffer 裡的位元組組成一行，
 * 收到完整一行（\r 或 \n 結尾）就解析執行。非阻塞。 */
void app_cli_poll(void);

#endif /* APP_CLI_H_ */
