#include "uart.h"

extern UART_HandleTypeDef huart1;
uint8_t uart1_rx_cnt = 0; // Receive buffer count

volatile bool is_data_ready_to_be_read = false;
volatile bool is_rx_full_completed = false;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		is_data_ready_to_be_read = true;
		is_rx_full_completed = true;
	}
}

volatile bool is_tx_completed = false;
volatile bool is_tx_full_completed = false;
/* Регистр TXE пуст */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		is_tx_completed = true;
		is_tx_full_completed = true;
	}
}

volatile bool is_rx_half_completed = false;
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		is_rx_half_completed = true;
	}
}

volatile bool is_tx_half_completed = true;
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		is_tx_half_completed = true;
	}
}

void receive_data_from_uart(char *rx_buff) {
	char rx_byte = ' ';
	char cnt;
	uint8_t isDataEnd = 0;

	memset(rx_buff, 0, strlen(rx_buff));
	uart1_rx_cnt = 0;
	while (!isDataEnd) {
		HAL_UART_Receive_IT(&huart1, (uint8_t*) &rx_byte, 1);

		if (is_data_ready_to_be_read) {
			is_data_ready_to_be_read = false;

			rx_buff[uart1_rx_cnt++] = rx_byte; // Receive byte

			if ((rx_buff[uart1_rx_cnt - 1] == 0x0A)
					&& (rx_buff[uart1_rx_cnt - 2] == 0x0D)) { // Chars end: CR+LF
				cnt = uart1_rx_cnt + '0';
				isDataEnd = 1;
			}
		}

	}

}

uint32_t index_byte = 0;
void receive_data_from_uart_dma(char *rx_buff, uint16_t rx_buff_len,
		char *final_buff, uint16_t final_buff_len) {

	uint16_t rx_half_len = rx_buff_len / 2;

	HAL_UART_Receive_DMA(&huart1, (uint8_t*) rx_buff, rx_buff_len);

	if (is_rx_half_completed) {
		is_rx_half_completed = false;

		memcpy(final_buff + index_byte, rx_buff, rx_half_len);
		memset(rx_buff, '\0', rx_half_len);
		index_byte += rx_half_len;
	}

	if (is_rx_full_completed) {
		is_rx_full_completed = false;

		memcpy(final_buff + index_byte, rx_buff + index_byte, rx_half_len);
		memset(rx_buff + rx_half_len, '\0', rx_half_len);
		index_byte += rx_half_len;
	}

	if (index_byte == final_buff_len) {
		index_byte = 0;
		memset(final_buff, '\0', final_buff_len);
	}

}

uint32_t index_tx_byte = 0;
void transmit_data_to_uart_dma(char *start_buff, uint16_t start_buff_len,
		char *tx_buff, uint16_t tx_buff_len) {

	while (index_tx_byte < start_buff_len) {
			is_tx_full_completed = false;

			memset(tx_buff, '\0', tx_buff_len);
			memcpy(tx_buff, start_buff + index_tx_byte, tx_buff_len);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*) tx_buff, tx_buff_len);
			while(!is_tx_full_completed) {}
			HAL_UART_DMAStop(&huart1);
			index_tx_byte += tx_buff_len;
	}

	if (index_tx_byte == start_buff_len) {
		index_tx_byte = 0;
		HAL_UART_DMAStop(&huart1);
		memset(tx_buff, '\0', tx_buff_len);
	}
}

void transmit_data_to_uart(char *tx_buff) {
	is_tx_completed = false;
	HAL_UART_Transmit_IT(&huart1, (uint8_t*) tx_buff, strlen(tx_buff));
	while (!is_tx_completed) {
	}
}

