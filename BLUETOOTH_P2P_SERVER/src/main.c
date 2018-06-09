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
#include "PingPong.h"
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>

volatile long g_systimer = 0;
//volatile uint8_t flag_online = 0;
//volatile uint8_t not_connected = 1;
long volume;
//void USART0_Handler();

//coisas SA
#define canal_generico_pino 1//canal 0 = PD30
//! DAC channel used for test
#define DACC_CHANNEL        0 // (PB13)
//! DAC register base for test
#define DACC_BASE           DACC
//! DAC ID for test
#define DACC_ID             ID_DACC


void SysTick_Handler() {
	g_systimer++;	
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);
	printf("kakaka \n");

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

}

/**
 * \brief AFEC interrupt callback function.
 */

PPBUF_DECLARE(buffer,120000);
volatile uint32_t buf = 0;

static void AFEC_Temp_callback(void){	
	/** The conversion data value */
	uint32_t g_ul_value = 0;

	g_ul_value = afec_channel_get_value(AFEC0, canal_generico_pino);
	

	// check swap
	if(ppbuf_get_full_signal(&buffer,false) == true) {
		ppbuf_get_full_signal(&buffer,true); // swap
	}
	
	ppbuf_insert_active(&buffer, &g_ul_value, sizeof(g_ul_value));
		
	/* gets the data on the pong buffer */
	ppbuf_remove_inactive(&buffer, &buf, sizeof(buf));	
	
    dacc_get_interrupt_status(DACC_BASE);
	if ((buffer.ping == 0)){
		dacc_write_conversion_data(DACC_BASE, buf/2, DACC_CHANNEL);
	}else{
		dacc_write_conversion_data(DACC_BASE, buf, DACC_CHANNEL);
	}
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer_get[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			timestart = g_systimer; // reset timeout
			buffer_get[counter++] = rx;
		}
	}
	buffer_get[counter] = 0x00;
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

static void config_ADC_TEMP(void){
/************************************* 
   * Ativa e configura AFEC
   *************************************/  
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);
  
	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_TIO_CH_0);
		
	AFEC0->AFEC_MR |= 3;
  
	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_1,	AFEC_Temp_callback, 1); 
   
	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, canal_generico_pino, &afec_ch_cfg);
  
	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, canal_generico_pino, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

	/* Selecina canal e inicializa conversão */  
	afec_channel_enable(AFEC0, canal_generico_pino);
}

static void config_DAC(void){
	/* Enable clock for DACC */
	sysclk_enable_peripheral_clock(DACC_ID);

	/* Reset DACC registers */
	dacc_reset(DACC_BASE);
	dacc_enable_channel(DACC_BASE, DACC_CHANNEL);
}

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	
	//PMC->PMC_SCER = 1 << 14;
	ul_tcclks = 1;
	
	tc_init(TC, TC_CHANNEL, ul_tcclks 
							| TC_CMR_WAVE /* Waveform mode is enabled */
							| TC_CMR_ACPA_SET /* RA Compare Effect: set */
							| TC_CMR_ACPC_CLEAR /* RC Compare Effect: clear */
							| TC_CMR_CPCTRG /* UP mode with automatic trigger on RC Compare */
	);
	
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq /8 );
	tc_write_ra(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq / 8 / 2);
	//tc_write_rc(TC, TC_CHANNEL, 3*65532/3);

	//tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	//NVIC_EnableIRQ((IRQn_Type) ID_TC);
//	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

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
	
	//coisas do pong
	/* Initialize the SAM system. */

	ioport_init();
		
	/* inicializa delay */
	delay_init(sysclk_get_cpu_hz());

		
	/* inicializa e configura adc */
	config_ADC_TEMP();
		
		
	//TC_init(TC0, ID_TC0, 0, 100000);
	
	char temp_volume[1024];
	char *str;
	
	while(1) {
		
		
		usart_get_string(USART0, temp_volume, 1024, 100);
		
		if(temp_volume[0] == 118){
			usart_log("before", temp_volume);
			volume  = strtol(temp_volume, &str, 10);
			usart_log("Volume", volume);
			usart_log("String", str);
		}

		
		delay_ms(1);

	}
}