/*
 Controle Remoto (client)
 Projeto Conectividade
 Professor Rafael Corsi e Eduardo Marossi
 Alunos Sabrina e Paulo 
 2018
 */
#include <asf.h>
#include <string.h>

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

volatile uint32_t g_systimer = 0;
volatile uint8_t encoderPosCount = 0;
volatile uint8_t pinALast;
volatile uint8_t flag_encoder = 0;



// Encoder decoder
#define EN_CLK_ID ID_PIOD
#define EN_CLK PIOD // Connected to CLK on KY040 encoder
#define EN_CLK_PIN 22
#define EN_CLK_PIN_MASK (1 <<  EN_CLK_PIN)

#define EN_DT_ID ID_PIOD
#define EN_DT PIOD // Connected to DT on KY040 encoder
#define EN_DT_PIN 21
#define EN_DT_PIN_MASK (1 <<  EN_DT_PIN)

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

/************************************************************************/
/* funcoes                                                              */
/*************************************************************************/

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
	usart_serial_options_t config;
	config.baudrate = 115200;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
}
/* 
void hm10_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	 // RX - PB0  TX - PB1 
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
} */

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
/*
int hm10_server_init(void) {
	char buffer_rx[128];
	usart_send_command(USART0, buffer_rx, 1000, "AT", 200);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 200);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 200);
	usart_send_command(USART0, buffer_rx, 1000, "AT+RESET", 400);	
	usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEServer", 400);
	usart_send_command(USART0, buffer_rx, 1000, "AT+ROLE0", 400);
	usart_log("hm10_server_init", buffer_rx);
} */

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
	pio_handler_set(EN_CLK,EN_CLK_ID, EN_CLK_PIN_MASK, PIO_IT_FALL_EDGE, Encoder_Handler);

	/* e configura sua prioridade  */
	NVIC_EnableIRQ(EN_CLK_ID);
	NVIC_SetPriority(EN_CLK_ID, 1);
	
	/*			PINO DT      */
	//config. pino DT em modo de entrada 
	pmc_enable_periph_clk(EN_DT_ID);
	pio_set_input(EN_DT, EN_DT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	//indica funcao (but_Handler) a ser chamada quando houver uma interrupo
	pio_enable_interrupt(EN_DT, EN_DT_PIN_MASK);// INTERRUPCAO
	pio_handler_set(EN_DT,EN_DT_ID, EN_DT_PIN_MASK, PIO_IT_FALL_EDGE, Encoder_Handler);

	//configura sua prioridade                       
	NVIC_EnableIRQ(EN_DT_ID);
	NVIC_SetPriority(EN_DT_ID, 0);
	
	}



static void Encoder_Handler(uint32_t id, uint32_t mask){

	//flag_encoder = 0;
	//sprintf(buffer, "%d \n", encoderPosCount);
	//usart_log("encoder", buffer);
	
	/*int temp = encoderPosCount++;
	char temp_str[5];
	itoa(temp, temp_str, 10);
	usart_put_string(UART3, temp_str);*/
	usart_put_string(USART1, "irq...\r\n");
			char buffer[54];
			sprintf(buffer, "flag before %d \n", flag_encoder);
			usart_put_string(USART1, buffer);

	flag_encoder = 1;
	
			
			sprintf(buffer, "flag after %d \n", flag_encoder);
			usart_put_string(USART1, buffer);
	//encoderPosCount++;
	
	/*
	volatile uint8_t aVal = pio_get(EN_CLK, PIO_INPUT,  EN_CLK_PIN_MASK);// digitalRead(pinA)?
	
		if (aVal != pinALast){ // Means the knob is rotating
			// if the knob is rotating, we need to determine direction
			// We do that by reading pin B.
			if (pio_get(PIOD, PIO_INPUT,  EN_DT_PIN_MASK)!= aVal){ // Means pin A Changed first  We're Rotating Clockwise
				encoderPosCount++;
				
			}
			else {// Otherwise B changed first and we're moving CCW
				encoderPosCount--;
				
			}
			pinALast = aVal;
		}
		*/

}


int main (void)
{
	board_init();
	sysclk_init();
	
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	delay_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_console();
		
	usart_put_string(USART1, "Inicializando...\r\n");
	/*
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hm10_config_server();
	hm10_server_init();
	*/
	usart_put_string(USART1, "Config HC05 Client...\r\n");
	hm10_config_client(); 
	hm10_client_init();
	char buffer[1024];
	
	pinALast = pio_get(EN_CLK, PIO_INPUT,  EN_CLK_PIN_MASK);
	
	Encoder_init();

	g_systimer = 0;
	encoderPosCount = 0;
	flag_encoder = 0;

	while(1) {
		//usart_put_string(UART3, "OI\n");
		//usart_get_string(UART3, buffer, 1024, 1000);
		//usart_log("main", buffer);
		
		sprintf(buffer, "flag %d \n", flag_encoder);
		usart_put_string(USART1, buffer);
		delay_ms(500);
		
		if(flag_encoder == 1){
			usart_put_string(USART1, "entrou...\r\n");

			sprintf(buffer, "%d \n", encoderPosCount);
			usart_log("encoder", buffer);
			int temp = encoderPosCount++;
			char temp_str[5];
			itoa(temp, temp_str, 10);
			usart_put_string(UART3, temp_str);
			flag_encoder = 0;
		}
		
		//sprintf(buffer, "%d \n", encoderPosCount);
		//usart_log("encoder", buffer);
		
	}
	
}
