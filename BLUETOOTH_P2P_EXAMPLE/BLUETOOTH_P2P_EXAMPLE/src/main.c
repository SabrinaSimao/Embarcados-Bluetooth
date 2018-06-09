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

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

volatile uint32_t g_systimer = 0;
volatile int32_t encoderPosCount = 50;
volatile uint32_t pinALast;
volatile uint32_t flag_but = 0;
volatile uint8_t but_number = 0;

#define BUT_PIO_ID			  ID_PIOA
#define BUT_PIO				  PIOA
#define BUT_PIN				  11
#define BUT_PIN_MASK			  (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79



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
	sprintf(buffer, "encoderPosCount %d \n", encoderPosCount);
	usart_put_string(USART1, buffer);
			
	volatile uint8_t aVal = pio_get(EN_CLK, PIO_INPUT,  EN_CLK_PIN_MASK);// digitalRead(pinA)?
	
	if (pio_get(PIOD, PIO_INPUT,  EN_DT_PIN_MASK)!= aVal) { // Means pin A Changed first  We're Rotating Clockwise
		if (encoderPosCount < 100){
			encoderPosCount++;
		}
		
	}
		else if (encoderPosCount > 1){// Otherwise B changed first and we're moving CCW
		encoderPosCount--;
		}
	
	usart_put_string(USART1, "mandando...\r\n");
	sprintf(buffer, "v %d \n", encoderPosCount);
	usart_log("encoder", buffer);
	usart_put_string(UART3, buffer);

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
	
/*static void Button_Handler(uint32_t id, uint32_t mask){
		usart_put_string(UART3, "!");
		flag_but = 1;
		
		char buffer[54];
		sprintf(buffer, "flag butt %d \n", flag_but);
		usart_put_string(USART1, buffer);
}
*/
static void Button_Handler(uint32_t id, uint32_t mask)
{
	 but_number++;
	 char but_temp[32];
	 
	 
	 if (but_number > 2){
		 but_number = 0;
	 }


	sprintf(but_temp, "i %d \n", but_number); // I = instruction
	usart_log("mandando instrucao de botao", but_temp);
	usart_put_string(UART3, but_temp);
	encoderPosCount = 50;

	 
}


int main (void)
{
	board_init();
	sysclk_init();
	ioport_init();
	
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	delay_init(sysclk_get_cpu_hz());
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_console();
	
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Client...\r\n");
	hm10_config_client(); 
	hm10_client_init();
	char buffer[1024];
	pinALast = pio_get(EN_CLK, PIO_INPUT,  EN_CLK_PIN_MASK);
	Encoder_init();
	BUT_init();

	
	g_systimer = 0;
	encoderPosCount = 50;
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
	
	//config_ADC_TEMP();		
	/* Output example information. */
	//puts(STRING_HEADER);

		
		
	/* incializa conversão ADC */
	//afec_start_software_conversion(AFEC0);
	BUT_init();
		

	// final ADC
	
	

	while(1) {
					
			//delay_ms(500);


			
		
		}
		
		//sprintf(buffer, "%d \n", encoderPosCount);
		//usart_log("encoder", buffer);
		
	}
