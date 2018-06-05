/**
 /*
 Controlador de Som (Server)
 Projeto Conectividade
 Professor Rafael Corsi e Eduardo Marossi
 Alunos Sabrina e Paulo 
 2018
 */

#include <asf.h>
#include <string.h>

volatile long g_systimer = 0;
//volatile uint8_t flag_online = 0;
//volatile uint8_t not_connected = 1;
volatile uint8_t volume;
//void USART0_Handler();


void SysTick_Handler() {
	g_systimer++;	
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			timestart = g_systimer; // reset timeout
			buffer[counter++] = rx;
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}

void config_console(void) {
	usart_serial_options_t config;
	config.baudrate = 115200;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
}

void hm10_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	sysclk_enable_peripheral_clock(ID_PIOB);
	//usart_init_rs232(USART0, &config, sysclk_get_peripheral_hz());


	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
	
	
	/* Ativa Clock e IRQ periferico USART0 */
	//sysclk_enable_peripheral_clock(ID_USART0);
	
	//usart_enable_interrupt(USART0, US_IER_RXRDY);
	//NVIC_SetPriority(ID_USART0, 1);
	//NVIC_EnableIRQ(ID_USART0);


}

/*void USART0_Handler(){
	
	char buffer[54];
	
	usart_get_string(USART0, buffer, 54, 1000);
	usart_put_string(USART1,"Entrou");
	
	if(not_connected){
		if (usart_get_string(USART0, buffer, 54, 1000) == "!\n"){
			flag_online = 1;
			not_connected = 0;
		} else{
			usart_put_string(USART1, "Recebi varias bosta\r\n");
			usart_put_string(USART0, "NA\n");	
		}
	}
}*/

/*
void hm10_config_client(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(UART3, &config);
	usart_enable_tx(UART3);
	usart_enable_rx(UART3);
	
	// RX - PD28 TX - PD30
	pio_configure(PIOD, PIO_PERIPH_A, (1 << 28), PIO_DEFAULT);
	pio_configure(PIOD, PIO_PERIPH_A, (1 << 30), PIO_DEFAULT);
}
*/
int hm10_server_init(void) {
	char buffer_rx[128];
	usart_send_command(USART0, buffer_rx, 1000, "AT", 200);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 200);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 200);
	usart_send_command(USART0, buffer_rx, 1000, "AT+RESET", 400);	
	usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEServer", 400);
	usart_send_command(USART0, buffer_rx, 1000, "AT+ROLE0", 400);
	usart_log("hm10_server_init", buffer_rx);
}
/*
int hm10_client_init(void) {
	char buffer_rx[128];
	usart_send_command(UART3, buffer_rx, 1000, "AT", 200);
	usart_send_command(UART3, buffer_rx, 1000, "AT", 200);
	usart_send_command(UART3, buffer_rx, 1000, "AT", 200);
	usart_send_command(UART3, buffer_rx, 1000, "AT+RESET", 400);
	usart_send_command(UART3, buffer_rx, 1000, "AT+NAMEClient", 400);
	usart_log("hm10_client_init", buffer_rx);
	usart_send_command(UART3, buffer_rx, 1000, "AT+IMME1", 400);
	usart_log("hm10_client_init", buffer_rx);
	usart_send_command(UART3, buffer_rx, 1000, "AT+ROLE1", 400);
	usart_log("hm10_client_init", buffer_rx);
	usart_send_command(UART3, buffer_rx, 1000, "AT+RESET", 800); // http://www.martyncurrey.com/hm-10-bluetooth-4ble-modules/
	usart_log("hm10_client_init", buffer_rx);
	usart_send_command(UART3, buffer_rx, 1000, "AT+DISC?", 10000); 
	usart_log("hm10_client_init", buffer_rx);
	usart_send_command(UART3, buffer_rx, 1000, "AT+CONN0", 1000);
	usart_log("hm10_client_init", buffer_rx);
	
}
*/
int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_console();
	
	
	
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hm10_config_server();
	hm10_server_init();
	
	char buffer[1024];
	
	
	while(1) {
		//usart_put_string(USART0, "Tchau\n");
		//usart_get_string(USART0, buffer, 1024, 1000);


		usart_get_string(USART0, buffer, 1024, 100);
		usart_log("Volume", buffer);
		sprintf(volume, "%d \n", buffer);
		delay_ms(1);

	}
	
	

	/* Insert application code here, after the board has been initialized. */
}
