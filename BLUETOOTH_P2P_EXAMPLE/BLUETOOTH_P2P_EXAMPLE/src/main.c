/*
 Controle Remoto (client)
 Projeto Conectividade
 Professor Rafael Corsi e Eduardo Marossi
 Alunos Sabrina e Paulo 
 2018
 */
#include <stdio.h>
#include <asf.h>
#include <string.h>
#include <assert.h>
#include "conf_dacc_sinewave_example.h"
#include "PingPong.h"

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

volatile uint32_t g_systimer = 0;
volatile int32_t encoderPosCount = 0;
volatile uint32_t pinALast;
volatile uint32_t flag_encoder = 0;
volatile uint32_t flag_but = 0;



// Encoder decoder
#define EN_CLK_ID ID_PIOD
#define EN_CLK PIOD // Connected to CLK on KY040 encoder
#define EN_CLK_PIN 22
#define EN_CLK_PIN_MASK (1 <<  EN_CLK_PIN)

#define EN_DT_ID ID_PIOD
#define EN_DT PIOD // Connected to DT on KY040 encoder
#define EN_DT_PIN 21
#define EN_DT_PIN_MASK (1 <<  EN_DT_PIN)

#define BUT_PIO_ID			  ID_PIOA
#define BUT_PIO				  PIOA
#define BUT_PIN				  11
#define BUT_PIN_MASK			  (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

#define buffer_siz 1024

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095UL)

/** The conversion data is done flag */
volatile bool is_conversion_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;

/* Canal do sensor de temperatura */
#define canal_generico_pino 0//canal 1 = PA21

volatile uint32_t buf = 0;
uint32_t dac_val;

int bufferTxindex = 0;

void make_buffer();
static void Encoder_Handler();
void config_console();
void Encoder_init();
int hm10_client_init();
void hm10_config_client();
void SysTick_Handler();
void usart_put_string();
void usart_log();
int usart_get_string();
void usart_send_command();
void BUT_init();
static void Button_Handler();

/************************************************************************/
/*									Defines			                    */
/*************************************************************************/
/** Header printf */
#define STRING_EOL    "\r"
#define STRING_HEADER "-- AFEC Temperature Sensor Example --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** Analog control value */
#if (SAMV70 || SAMV71 || SAME70 || SAMS70)
#define DACC_ANALOG_CONTROL (DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02))
#else
#define DACC_ANALOG_CONTROL (DACC_ACR_IBCTLCH0(0x02) \
| DACC_ACR_IBCTLCH1(0x02) \
| DACC_ACR_IBCTLDACCORE(0x01))
#endif

/** The maximal sine wave sample data (no sign) */
#define MAX_DIGITAL   (0x7ff)
/** The maximal (peak-peak) amplitude value */
#define MAX_AMPLITUDE (DACC_MAX_DATA)
/** The minimal (peak-peak) amplitude value */
#define MIN_AMPLITUDE (100)

/** SAMPLES per cycle */
#define SAMPLES (100)

/** Default frequency */
#define DEFAULT_FREQUENCY 1000
/** Min frequency */
#define MIN_FREQUENCY   200
/** Max frequency */
#define MAX_FREQUENCY   3000

/** Invalid value */
#define VAL_INVALID     0xFFFFFFFF

//PPBUF_DECLARE(buffer,buffer_siz);
/************************************************************************/
/* funcoes    ADC                                                       */
/*************************************************************************/
void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);
	//printf("kakaka \n");

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

}

/**
 * \brief AFEC interrupt callback function.
 */


static void AFEC_Temp_callback(void){


	uint32_t status;
	uint32_t data = 0;
	
	/*gets data from afec*/
	data = afec_channel_get_value(AFEC0, canal_generico_pino);
	
	/*
	// check swap
	if(ppbuf_get_full_signal(&buffer,false) == true) {
		ppbuf_get_full_signal(&buffer,true); // swap
	}
	
	ppbuf_insert_active(&buffer, &data, sizeof(data));
		
	/* gets the data on the pong buffer */
	//ppbuf_remove_inactive(&buffer, &buf, sizeof(buf));	
	
	/*writes on dacc*/
	status = dacc_get_interrupt_status(DACC_BASE);
	dacc_write_conversion_data(DACC_BASE, data, DACC_CHANNEL);
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
	afec_channel_set_analog_offset(AFEC0, canal_generico_pino, 0);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);
	//pio_pull_down(PIOA, (1u) << 21, 1);

	/* Selecina canal e inicializa conversão */  
	afec_channel_enable(AFEC0, canal_generico_pino);
}

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

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
/* funcoes                                                              */
/*************************************************************************/


/*! Convert wave data to DACC value
 *  Put the sinewave to an offset of max_amplitude/2.
 *  \param wave          Waveform data
 *  \param amplitude     Amplitude value
 *  \param max_digital   Maximal digital value of input data (no sign)
 *  \param max_amplitude Maximal amplitude value
 */
#define wave_to_dacc(wave, amplitude, max_digital, max_amplitude) \
	(((int)(wave) * (amplitude) / (max_digital)) + (max_amplitude / 2))

/** Current g_ul_index_sample */
uint32_t g_ul_index_sample = 0;
/** Frequency */
uint32_t g_ul_frequency = 0;
/** Amplitude */
int32_t g_l_amplitude = 0;

/** Waveform selector */
uint8_t g_uc_wave_sel = 0;

/** 100 points of sinewave samples, amplitude is MAX_DIGITAL*2 */
const int16_t gc_us_sine_data[SAMPLES] = {
	0x0,   0x080, 0x100, 0x17f, 0x1fd, 0x278, 0x2f1, 0x367, 0x3da, 0x449,
	0x4b3, 0x519, 0x579, 0x5d4, 0x629, 0x678, 0x6c0, 0x702, 0x73c, 0x76f,
	0x79b, 0x7bf, 0x7db, 0x7ef, 0x7fb, 0x7ff, 0x7fb, 0x7ef, 0x7db, 0x7bf,
	0x79b, 0x76f, 0x73c, 0x702, 0x6c0, 0x678, 0x629, 0x5d4, 0x579, 0x519,
	0x4b3, 0x449, 0x3da, 0x367, 0x2f1, 0x278, 0x1fd, 0x17f, 0x100, 0x080,

	-0x0,   -0x080, -0x100, -0x17f, -0x1fd, -0x278, -0x2f1, -0x367, -0x3da, -0x449,
	-0x4b3, -0x519, -0x579, -0x5d4, -0x629, -0x678, -0x6c0, -0x702, -0x73c, -0x76f,
	-0x79b, -0x7bf, -0x7db, -0x7ef, -0x7fb, -0x7ff, -0x7fb, -0x7ef, -0x7db, -0x7bf,
	-0x79b, -0x76f, -0x73c, -0x702, -0x6c0, -0x678, -0x629, -0x5d4, -0x579, -0x519,
	-0x4b3, -0x449, -0x3da, -0x367, -0x2f1, -0x278, -0x1fd, -0x17f, -0x100, -0x080
};


void SysTick_Handler() {
	g_systimer++;
	
	//Coisas do dacc
	//uint32_t status;
	
	
	//status = dacc_get_interrupt_status(DACC_BASE);

	//If ready for new data
	/*
	if ((status & DACC_ISR_TXRDY0) == DACC_ISR_TXRDY0) {
		g_ul_index_sample++;
		if (g_ul_index_sample >= SAMPLES) {
			g_ul_index_sample = 0;
		}
		dac_val = g_uc_wave_sel ?
				((g_ul_index_sample > SAMPLES / 2) ? 0 : MAX_AMPLITUDE)
				: wave_to_dacc(gc_us_sine_data[g_ul_index_sample],
					 g_l_amplitude,
					 MAX_DIGITAL * 2, MAX_AMPLITUDE);
		*/

		//dacc_write_conversion_data(DACC_BASE, dac_val, DACC_CHANNEL);
		
	
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
			//timestart = g_systimer; // reset timeout
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
	sysclk_enable_peripheral_clock(ID_USART1);

	usart_serial_options_t config;
	config.baudrate = 115200;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
}

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
	usart_send_command(UART3, buffer_rx, 1000, "AT+DISC?", 1000); 
	usart_log("hm10_client_init", buffer_rx);
	usart_send_command(UART3, buffer_rx, 1000, "AT+COND43639D8BD1D", 1000); //D43639D8BD1D
	usart_log("hm10_client_init", buffer_rx);
	
}

void Encoder_init(void){
	/* config. pino CLK em modo de entrada */
	pmc_enable_periph_clk(EN_CLK_ID);
	pio_set_input(EN_CLK, EN_CLK_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupo */
	pio_enable_interrupt(EN_CLK, EN_CLK_PIN_MASK);// INTERRUPCAO
	pio_handler_set(EN_CLK,EN_CLK_ID, EN_CLK_PIN_MASK, PIO_IT_FALL_EDGE || PIO_IT_RISE_EDGE, Encoder_Handler);

	/* e configura sua prioridade  */
	NVIC_EnableIRQ(EN_CLK_ID);
	NVIC_SetPriority(EN_CLK_ID, 1);
	
	}


static void Encoder_Handler(uint32_t id, uint32_t mask){
	
	char buffer[42];
	sprintf(buffer, "flag before encoder %d \n", flag_encoder);
	usart_put_string(USART1, buffer);
			
	volatile uint8_t aVal = pio_get(EN_CLK, PIO_INPUT,  EN_CLK_PIN_MASK);// digitalRead(pinA)?
	
	if (pio_get(PIOD, PIO_INPUT,  EN_DT_PIN_MASK)!= aVal) { // Means pin A Changed first  We're Rotating Clockwise
		if (encoderPosCount < 100){
			encoderPosCount++;
		}
		
	}
	else if (encoderPosCount > 0){// Otherwise B changed first and we're moving CCW
		encoderPosCount--;
		}
		
		
	flag_encoder = 1;
	pinALast = aVal;
	
}

void BUT_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupo */
	pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);// INTERRUPCAO
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button_Handler);

	/* habilita interrupco do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 1);
	}
	
static void Button_Handler(uint32_t id, uint32_t mask){
		usart_put_string(UART3, "!");
		flag_but = 1;
		
		char buffer[54];
		sprintf(buffer, "flag butt %d \n", flag_but);
		usart_put_string(USART1, buffer);
}


int main (void)
{
	board_init();
	sysclk_init();
	ioport_init();
	
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	//delay_init(sysclk_get_cpu_hz());
	//SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	//config_console();
		
	//usart_put_string(USART1, "Inicializando...\r\n");
	//usart_put_string(USART1, "Config HC05 Client...\r\n");
	//hm10_config_client(); 
	//hm10_client_init();
	//char buffer[1024];
	//pinALast = pio_get(EN_CLK, PIO_INPUT,  EN_CLK_PIN_MASK);
	//Encoder_init();
	//BUT_init();

	
	g_systimer = 0;
	//encoderPosCount = 0;
	//flag_encoder = 0;
	//flag_but = 1;
	//
	pmc_enable_periph_clk(ID_PIOA);
	pio_set_output(PIOA, 1, 0, 0, 0);
	pio_set_peripheral(PIOA, PIO_PERIPH_B, 1);
	
	// ADC
	/* inicializa e configura adc */
	
	

	/* Enable clock for DACC */
	sysclk_enable_peripheral_clock(DACC_ID);

	/* Reset DACC registers */
	dacc_reset(DACC_BASE);
	dacc_enable_channel(DACC_BASE, DACC_CHANNEL);

	//dacc_write_conversion_data(DACC_BASE, 1024,DACC_CHANNEL);
	
	config_ADC_TEMP();		
	/* Output example information. */
	//puts(STRING_HEADER);
		
	TC_init(TC0, ID_TC0, 0, 100000);
		
		
	/* incializa conversão ADC */
	//afec_start_software_conversion(AFEC0);
		
		

	// final ADC

	while(1) {
		
		//if (flag_but){
		//	//usart_put_string(UART3, "OI\n");
		//	//usart_get_string(UART3, buffer, 1024, 1000);
		//	//usart_log("main", buffer);
		//	
		//	// AFEC
		//	if(is_conversion_done == true) {
		//		is_conversion_done = false;
		//		make_buffer(g_ul_value);
		//		if (!not_full){
		//			for(uint32_t i = 0; i < 16; i++){
		//				//dacc_write_conversion_data(DACC_BASE, bufferA[i], DACC_CHANNEL);//temporario
		//				printf("BufferA : %d \r\n", bufferA[i]);
		//				sprintf(buffer, "flag before %d \n", bufferA[i]);
		//				usart_put_string(USART1, buffer);
		//			}
		//		}
		//				
		//		if(full_B){
		//			for(uint32_t i = 0; i < 16; i++){
		//				//dacc_write_conversion_data(DACC_BASE, bufferB[i], DACC_CHANNEL);//temporario
		//				printf("BufferB : %d \r\n", bufferB[i]);
		//				sprintf(buffer, "flag before %d \n", bufferB[i]);
		//				usart_put_string(USART1, buffer);
		//			}
		//		}
		//		//printf("Temp : %d \r\n", convert_adc_to_temp(g_ul_value));
		//				
		//		//afec_start_software_conversion(AFEC0);
		//		delay_ms(1);
		//	}
		//	
		//	//Bluetooth
		//	/*		
		//	sprintf(buffer, "flag %d \n", flag_encoder);
		//	usart_put_string(USART1, buffer);
		//	delay_ms(500);
		//
		//	if(flag_encoder == 1){
		//		usart_put_string(USART1, "entrou...\r\n");
		//
		//		sprintf(buffer, "%d \n", encoderPosCount);
		//		usart_log("encoder", buffer);
		//		int temp = encoderPosCount;
		//		char temp_str[5];
		//		itoa(temp, temp_str, 10);
		//		usart_put_string(UART3, temp_str);
		//		flag_encoder = 0;
		//	}
		//	*/
		//
		//}
		//
		////sprintf(buffer, "%d \n", encoderPosCount);
		////usart_log("encoder", buffer);
		
	}
	
}
