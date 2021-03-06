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
#include <math.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "PingPong.h"

volatile long g_systimer = 0;
//volatile uint8_t flag_online = 0;
//volatile uint8_t not_connected = 1;
//void USART0_Handler();

//coisas SA
#define canal_generico_pino 0//canal 0 = PD30 canal 1 = PA21
//! DAC channel used for test
#define DACC_CHANNEL        0 // (PB13)
//! DAC register base for test
#define DACC_BASE           DACC
//! DAC ID for test
#define DACC_ID             ID_DACC

#define TEST(f) {.test_function=f, .test_name=#f}

/************************************************************************/
/*        inicializando funcoes                                         */
/************************************************************************/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void Softning();
void Hard_clipping();
void Volume();

typedef struct {
	void (*test_function)();
	char test_name[100];
} test_data;

test_data t[] = {TEST(Softning), TEST(Hard_clipping), TEST(Volume)};

uint32_t corte_filtro = 4000;
long volume = 50;
uint32_t g_ul_value_old = 0;
uint32_t temp;
uint32_t g_ul_value = 0;
uint32_t corte_alto = 3000;
uint32_t corte_baixo = 300;
uint8_t funcao_escolhida = 0;


 void Softning(){
	float constante = volume/1000;
	temp = g_ul_value;
	g_ul_value = (int) ((float) g_ul_value * (float) g_ul_value_old* constante) + g_ul_value ;
	g_ul_value_old = temp;

}

 void Hard_clipping(){
	g_ul_value = g_ul_value *4;
	float corte = log2(volume);
	uint32_t corte_alto = (int)(corte * 500);
	uint32_t corte_baixo = (int)(corte * 200);
	if (g_ul_value > corte_alto){
		g_ul_value = corte_alto;
	}
	if (g_ul_value < corte_baixo){
		g_ul_value = corte_baixo;
	}
	g_ul_value = g_ul_value / 4;

}

 void Volume(){
	float volume_temp = volume/100;
	g_ul_value  = (int) ((float) g_ul_value * volume_temp);

}

void SysTick_Handler() {
	g_systimer++;	
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupo foi satisfeita.
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
	
	g_ul_value = afec_channel_get_value(AFEC0, canal_generico_pino);
	
	//t[funcao_escolhida].test_function();
	
	
	// check swap
	if(ppbuf_get_full_signal(&buffer,false) == true) {
		ppbuf_get_full_signal(&buffer,true); // swap
	}
	
	ppbuf_insert_active(&buffer, &g_ul_value, sizeof(g_ul_value));
	
	/* gets the data on the pong buffer */
	ppbuf_remove_inactive(&buffer, &buf, sizeof(buf));
	
	dacc_get_interrupt_status(DACC_BASE);
	
	if ((buffer.ping == 0)){
		dacc_write_conversion_data(DACC_BASE, buf/4, DACC_CHANNEL);
	}
	else{
		dacc_write_conversion_data(DACC_BASE, buf/16, DACC_CHANNEL);
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
	
	


}



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
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_0,	AFEC_Temp_callback, 1); 
   
	/*** Configuracao especfica do canal AFEC ***/
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

	/* Selecina canal e inicializa converso */  
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

	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupco no RC compare */
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


	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

int main (void)
{
	board_init();
	sysclk_init();
	//delay_init();
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
	config_DAC();
		
	TC_init(TC0, ID_TC0, 0, 100000);
	
	char temp_instrucao[1024];
	char *str;
	
	while(1) {
		
		
		usart_get_string(USART0, temp_instrucao, 1024, 1);
		
		if(temp_instrucao[0] == 105){ // checa se  uma instruo tipo i(intruo_geral)
			usart_log("antes", temp_instrucao);
			funcao_escolhida = strtol(temp_instrucao, &str, 10);
			usart_log("Volume", funcao_escolhida);
			usart_log("String", str);
		}
		
		if(temp_instrucao[0] == 118){ // checa se  uma instruo tipo v(volume)
			usart_log("antes", temp_instrucao);
			volume  = strtol(temp_instrucao, &str, 10);
			usart_log("Volume", volume);
			usart_log("String", str);
		}

		
		delay_ms(100);

	}
}