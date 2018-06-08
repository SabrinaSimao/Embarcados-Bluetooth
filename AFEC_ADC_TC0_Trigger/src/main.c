/** 
 * 12- ADC
 * Rafael Corsi @ insper.edu.br
 * Abril 2017
 * 
 * Configura o ADC do SAME70 para fazer leitura
 * do sensor de temperatura interno
 */

/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "asf.h"
#include "PingPong.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/** Header printf */
#define STRING_EOL    "\r"
#define STRING_HEADER "-- AFEC Temperature Sensor Example --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL


#define canal_generico_pino 1//canal 0 = PD30

//! DAC channel used for test
#define DACC_CHANNEL        0 // (PB13)
//! DAC register base for test
#define DACC_BASE           DACC
//! DAC ID for test
#define DACC_ID             ID_DACC

/************************************************************************/
/*                                                                      */
/************************************************************************/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);

/************************************************************************/
/* Callbacks / Handler                                                 */
/************************************************************************/
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

PPBUF_DECLARE(buffer,12000);
volatile uint32_t buf = 0;
volatile uint8_t passa_baixa_ativo = 0;
uint32_t corte_filtro = 2800;

static void AFEC_Temp_callback(void){	
	/** The conversion data value */
	uint32_t g_ul_value = 0;

	g_ul_value = afec_channel_get_value(AFEC0, canal_generico_pino);
	if(passa_baixa_ativo){
		if (g_ul_value>corte_filtro){
			g_ul_value = corte_filtro;
		}
	}

	// check swap
	//if(ppbuf_get_full_signal(&buffer,false) == true) {
	//	ppbuf_get_full_signal(&buffer,true); // swap
	//}
	
	//ppbuf_insert_active(&buffer, &g_ul_value, sizeof(g_ul_value));
		
	/* gets the data on the pong buffer */
	//ppbuf_remove_inactive(&buffer, &buf, sizeof(buf));	
	dacc_write_conversion_data(DACC_BASE, g_ul_value, DACC_CHANNEL);
	
    dacc_get_interrupt_status(DACC_BASE);
	/*
	if ((buffer.ping == 0)){
		dacc_write_conversion_data(DACC_BASE, buf/4, DACC_CHANNEL);
	}else{
		dacc_write_conversion_data(DACC_BASE, buf, DACC_CHANNEL);
	}*/
}

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/**
 * \brief Configure UART console.
 * BaudRate : 115200
 * 8 bits
 * 1 stop bit
 * sem paridade
 */
static void configure_console(void)
{
 
	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
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


/************************************************************************/
/* Main                                                                 */
/************************************************************************/
/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{

	/* Initialize the SAM system. */
	sysclk_init();
	ioport_init();
	board_init();
  
	/* inicializa delay */
	delay_init(sysclk_get_cpu_hz());

	/* inicializa console (printf) */
	configure_console();
	
	/* inicializa e configura adc */
	config_ADC_TEMP();
  
	/* Output example information. */
	puts(STRING_HEADER);
	
	TC_init(TC0, ID_TC0, 0, 100000);

	config_DAC();

	while (1) {
		
	}
}
