/*
 * uart_util.c
 *
 *  Created on: Jul 29, 2016
 *      Author: strv
 */

#include "uart_util.h"
#include "xprintf.h"
#include <string.h>

static char rx_buff[UU_BUFF_LEN];
static char tx_buff[UU_BUFF_LEN];
static const uint32_t buff_end = UU_BUFF_LEN - 1;
static uint16_t rx_buff_wp;
static uint16_t rx_buff_rp;
static uint16_t tx_buff_wp;
static uint16_t tx_buff_rp;
static bool tx_ore, rx_ore;
static UU_ConsoleCommand commands[UU_MAX_COMMAND_NUM];
static uint16_t command_cnt;

static int32_t str2upper(char* str);

void uu_init(void){
	rx_buff_wp = 0;
	rx_buff_rp = 0;
	tx_buff_wp = 0;
	tx_buff_rp = 0;
	tx_ore = false;
	rx_ore = false;
	command_cnt = 0;
	xdev_out(uu_putc);
	xdev_in(uu_getc);

	UU_UART->CR1 |= USART_CR1_RXNEIE;
}

void uu_putc(unsigned char c){
	static unsigned char prev_c = 0;
	static uint16_t tmp_p;
	if((prev_c == '\r') && (c != '\n')){
		uu_putc('\n');
	}else if((c == '\n') && (prev_c != '\r')){
		uu_putc('\r');
	}
	tmp_p = (tx_buff_wp + 1) & (UU_BUFF_LEN - 1);
	if(tmp_p != tx_buff_rp){
		UU_UART->CR1 &= ~USART_CR1_TXEIE;
		tx_buff[tmp_p] = c;
		tx_buff_wp = tmp_p;
		UU_UART->CR1 |= USART_CR1_TXEIE;
		prev_c = c;
	}else{
		tx_ore = true;
	}
}

unsigned char uu_getc(void){
	char c = 0;
	if(rx_buff_rp != rx_buff_wp){
		UU_UART->CR1 &= ~USART_CR1_RXNEIE;
		rx_buff_rp++;
		rx_buff_rp &= (UU_BUFF_LEN - 1);
		c = rx_buff[rx_buff_rp];
		UU_UART->CR1 |= USART_CR1_RXNEIE;
	}
	return c;
}

bool uu_rx_buff_ore(void){
	bool b = rx_ore;
	rx_ore = false;
	UU_UART->CR1 |= USART_CR1_RXNEIE;
	return b;
}

bool uu_tx_buff_ore(void){
	bool b = tx_ore;
	tx_ore = false;
	return b;
}

bool uu_tx_busy(void){
	if(tx_buff_rp != tx_buff_wp){
		return true;
	}
	return false;
}

void uu_rx_buff_flush(void){
	UU_UART->CR1 &= ~USART_CR1_RXNEIE;
	rx_buff_wp = 0;
	rx_buff_rp = 0;
	UU_UART->CR1 |= USART_CR1_RXNEIE;
}

void uu_tx_buff_flush(void){
	UU_UART->CR1 &= ~USART_CR1_TXEIE;
	tx_buff_wp = 0;
	tx_buff_rp = 0;
	UU_UART->CR1 |= USART_CR1_TXEIE;
}

void UU_IRQ_Handler(void){
	static char rx_tmp;
	static uint16_t tmp_p;
	static uint32_t int_state;
	int_state = UU_UART->ISR;

	if(int_state & USART_ISR_RXNE){
		tmp_p = (rx_buff_wp + 1) & buff_end;
		rx_tmp = UU_UART->RDR;
		if(tmp_p != rx_buff_rp){
			rx_buff[tmp_p] = rx_tmp;
			rx_buff_wp = tmp_p;
		}else{
			rx_ore = true;
			UU_UART->CR1 &= ~USART_CR1_RXNEIE;
		}
	}else if(int_state & USART_ISR_TXE){
		if(tx_buff_rp != tx_buff_wp){
			tx_buff_rp++;
			tx_buff_rp &= (UU_BUFF_LEN - 1);
			UU_UART->TDR = tx_buff[tx_buff_rp];
		}else{
			UU_UART->CR1 &= ~USART_CR1_TXEIE;
		}
	}

	if(int_state & USART_ISR_ORE){
		rx_ore = true;
		UU_UART->ICR = USART_ICR_ORECF;
	}
	if(int_state & USART_ISR_FE){
		UU_UART->ICR = USART_ICR_FECF;
	}
}

void uu_proc_command(void){
	static char str[UU_CSL_STR_LEN] = {};
	uint32_t str_ofst;
	static char cmd[UU_CSL_CMD_LEN] = {'?'};
	static char* parg;
	static int32_t argv[UU_CSL_ARG_MAX];
	static int32_t argc = 0;
	uint32_t str_len = 0;
	uint32_t cmd_len = 0;
	bool done = false;

	if(uu_rx_buff_ore()){
		uu_rx_buff_flush();
		memset(str, '\0', UU_CSL_STR_LEN);
		xputs("Over run detected\r\n");
		return;
	}

	str_ofst = strlen(str);
 	if(xgets(str + str_ofst, UU_CSL_STR_LEN - str_ofst)){
		str_len = strlen(str);
		cmd_len = strcspn(str, " ");
		if(cmd_len >= UU_CSL_CMD_LEN){
			xputs("Too long command\r\n");
			goto proc_out;
		}else if (cmd_len > 0) {
			strncpy(cmd, str, cmd_len);
			cmd[cmd_len] = '\0';
			parg = &str[cmd_len + 1];
			argc = 0;
			while ((parg < str + str_len)) {
				if (!xatoi(&parg, &argv[argc])) {
					argv[argc] = 0;
					break;
				} else {
					argc++;
					if(argc == UU_CSL_ARG_MAX){
						xputs("Too much arguments\r\n");
						goto proc_out;
					}
				}
			}
		}

		str2upper(cmd);
		if(!strcmp(cmd, "TEST")) {
			xprintf("Test command\r\n");
			for (uint16_t i = 0; i < argc; i++) {
				xprintf("Arg%d:%d\r\n", i, argv[i]);
			}
			done = true;
		}else if(!strcmp(cmd, "?")){
			xputs("Built in commands\r\n");
			xputs("Command : ?\r\n");
			xputs("Usage : ?\r\n");
			xputs("Show this help.\r\n");
			xputs("--------------------\r\n");
			xputs("Command : TEST\r\n");
			xputs("Usage : TEST arg1 arg2 ...\r\n");
			xputs("Console test command. The command echo back arguments.\r\n");
			xputs("\r\nAddon commands\r\n");
			for(uint16_t i = 0; i < command_cnt; i++){
				if(i != 0){
					xputs("--------------------\r\n");
				}
				xputs("Command : ");
				xputs(commands[i].cmd_name);
				xputs("\r\n");
				xputs("Usage : ");
				xputs(commands[i].help_msg);
				xputs("\r\n");
			}
			done = true;
		}else {
			for(uint16_t i = 0; i < command_cnt; i++){
				if(!strcmp(cmd, commands[i].cmd_name)){
					if(!commands[i].func(argc, argv)){
						xputs(commands[i].help_msg);
						xputs("\r\n");
					}
					done = true;
					break;
				}
			}
		}
		if(!done){
			xputs("UNDEF\r\n");
		}

proc_out:
		xputs(">");
		memset(str, '\0', str_len);
	}else{
		if(strlen(str) == UU_CSL_STR_LEN - 1){
			memset(str, '\0', UU_CSL_STR_LEN);
			xputs("Too long command\r\n");
		}
	}
}

bool uu_push_command(UU_ConsoleCommand* pcmd){
	if(command_cnt >= UU_MAX_COMMAND_NUM){
		return false;
	}

	commands[command_cnt].cmd_name = pcmd->cmd_name;
	commands[command_cnt].func = pcmd->func;
	commands[command_cnt].help_msg = pcmd->help_msg;

	command_cnt++;
	return true;
}

static int32_t str2upper(char* str) {
	int32_t i;
	for (i = 0; *(str + i) != '\0'; i++) {
		if (*(str + i) >= 'a' && *(str + i) <= 'z')
			*(str + i) -= 0x20;
	}
	return i;
}
