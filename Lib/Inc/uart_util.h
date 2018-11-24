/*
 * uart_util.h
 *
 *  Created on: Jul 29, 2016
 *      Author: strv
 */

#ifndef UART_UTIL_H_
#define UART_UTIL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "stm32f3xx.h"

#define	UU_BUFF_LEN		(1 << 11)
#define	UU_UART			USART2
#define	UU_IRQ_Handler	USART2_IRQHandler
#define	UU_NL_TXT		'\r'
#define	UU_MAX_COMMAND_NUM	(64)
#define UU_CSL_STR_LEN	(128)
#define UU_CSL_CMD_LEN	(16)
#define UU_CSL_ARG_MAX	(8)

typedef struct{
	const char* cmd_name;
	bool (*func)(int32_t argc,int32_t* argv);
	const char* help_msg;
}UU_ConsoleCommand;

void uu_init(void);
void uu_putc(unsigned char c);
unsigned char uu_getc(void);
bool uu_rx_buff_ore(void);
bool uu_tx_buff_ore(void);
void uu_rx_buff_flush(void);
void uu_tx_buff_flush(void);
bool uu_tx_busy(void);
void UU_IRQ_Handler(void);

void uu_proc_command(void);
bool uu_push_command(UU_ConsoleCommand* pcmd);

#endif /* UART_UTIL_H_ */
