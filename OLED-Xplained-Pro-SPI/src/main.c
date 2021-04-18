#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"


// LED placa
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// LED 1
#define LED_1_PIO      PIOA
#define LED_1_PIO_ID   ID_PIOA
#define LED_1_IDX      0
#define LED_1_IDX_MASK (1 << LED_1_IDX)

//LED 2
#define LED_2_PIO      PIOC
#define LED_2_PIO_ID   ID_PIOC
#define LED_2_IDX      30
#define LED_2_IDX_MASK (1 << LED_2_IDX)

//LED 3
#define LED_3_PIO      PIOB
#define LED_3_PIO_ID   ID_PIOB
#define LED_3_IDX      2
#define LED_3_IDX_MASK (1 << LED_3_IDX)

// Botão 1
#define BUT_PIO PIOD
#define BUT_PIO_ID ID_PIOD
#define BUT_IDX 28
#define BUT_IDX_MASK (1 << BUT_IDX)

// Botão 2
#define BUT_2_PIO PIOC
#define BUT_2_PIO_ID ID_PIOC
#define BUT_2_IDX 31
#define BUT_2_IDX_MASK (1 << BUT_2_IDX)

// Botão 3
#define BUT_3_PIO PIOA
#define BUT_3_PIO_ID ID_PIOA
#define BUT_3_IDX 19
#define BUT_3_IDX_MASK (1 << BUT_3_IDX)


// Botão 0
#define BUT_0_PIO			  PIOA
#define BUT_0_PIO_ID		  ID_PIOA
#define BUT_0_PIO_IDX		  11
#define BUT_0_PIO_IDX_MASK (1u << BUT_0_PIO_IDX)

volatile char but_1_flag = 0;
volatile char but_2_flag = 0;
volatile char but_3_flag = 0;

volatile char flag_tc1 = 0;
volatile char flag_tc2 = 0;
volatile char flag_tc3 = 0;

volatile Bool f_rtt_alarme = false;

volatile Bool authorize = false;

volatile char s = 0;

volatile int count = 0;

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

void but_1_callback(void){
	but_1_flag = !but_1_flag;
}

void but_2_callback(void){
	but_2_flag = !but_2_flag;
}

void but_3_callback(void){
	but_3_flag = !but_3_flag;
}


void pisca_led(int n, int t){
	for (int i=0;i<n;i++){
		pio_clear(LED_1_PIO, LED_1_IDX_MASK);
		delay_ms(t);
		pio_set(LED_1_PIO, LED_1_IDX_MASK);
		delay_ms(t);
	}
}

void pisca_led_2(int n, int t){
	for (int i=0;i<n;i++){
		pio_clear(LED_2_PIO, LED_2_IDX_MASK);
		delay_ms(t);
		pio_set(LED_2_PIO, LED_2_IDX_MASK);
		delay_ms(t);
	}
}


void pisca_led_3(int n, int t){
	for (int i=0;i<n;i++){
		pio_clear(LED_3_PIO, LED_3_IDX_MASK);
		delay_ms(t);
		pio_set(LED_3_PIO, LED_3_IDX_MASK);
		delay_ms(t);
	}
}


void LED_1_init(int estado){
	pmc_enable_periph_clk(LED_1_PIO_ID);
	pio_set_output(LED_1_PIO, LED_1_IDX_MASK, estado, 0, 0 );
};

void LED_2_init(int estado){
	pmc_enable_periph_clk(LED_2_PIO_ID);
	pio_set_output(LED_2_PIO, LED_2_IDX_MASK, estado, 0, 0 );
};

void LED_3_init(int estado){
	pmc_enable_periph_clk(LED_3_PIO_ID);
	pio_set_output(LED_3_PIO, LED_3_IDX_MASK, estado, 0, 0 );
};


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
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
  NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
	
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
}


void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		//f_rtt_alarme = false;
		// BLINK Led
		count += 1;
		
		
		
	}

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		// pin_toggle(LED_PIO, LED_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;					 // flag RTT alarme
		authorize = !authorize;
		//pin_toggle(LED_PIO_2, LED_2_IDX_MASK);
		
			
		//pisca_led(1, 200);
		//pisca_led_2(1, 200);
		//pisca_led_3(1, 200);
		
	}
}


void enable(){
	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pmc_enable_periph_clk(BUT_3_PIO_ID);


	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_IDX_MASK, PIO_IT_RISE_EDGE, but_1_callback);
	pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_RISE_EDGE, but_2_callback);
	pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_RISE_EDGE, but_3_callback);



	// Ativa interrupção
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
	pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
	pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);



	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4
	
	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4);
	
	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4);
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc1 = 1;
	flag_tc2 = 1;
	flag_tc3 = 1;
}

void TC6_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC2, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc2 = 1;
	
}

void TC3_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC1, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc3 = 1;
}



int main (void)
{
	board_init();
	sysclk_init();
	delay_init();

  // Init OLED
	gfx_mono_ssd1306_init();
	
	TC_init(TC0, ID_TC1, 1, 4);
	TC_init(TC1, ID_TC3, 0, 4);
	TC_init(TC2, ID_TC6, 0, 4);
	
	enable();
	
	LED_1_init(1);
	LED_2_init(1);
	LED_3_init(1);
	
	f_rtt_alarme = true;
  
  // Escreve na tela um circulo e um texto
	//gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
  //gfx_mono_draw_string("mundo", 50,16, &sysfont);

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		
		if(flag_tc1){
			if(but_1_flag){
				if(authorize){
					pisca_led(1, 200);
				}
			}

		}
		//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		
		
		if(flag_tc2){
			if(but_2_flag){
				if(authorize){
					pisca_led_2(1, 10);	
				}
			}
		}
		//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		if(flag_tc3){
			if(but_3_flag){
				if(authorize){
					pisca_led_3(1, 1000);
				}
			}
		}
		
		
		
		if(f_rtt_alarme){
			uint16_t pllPreScale = (int) (((float) 32768) / 4);
			uint32_t irqRTTvalue = 20;
			
			
			
			// reinicia RTT para gerar um novo IRQ
			RTT_init(pllPreScale, irqRTTvalue);
			
			f_rtt_alarme = false;
		}
		
		if(count == 4){
			gfx_mono_draw_string("*", 16,16, &sysfont);
		}
		if(count == 8){
			gfx_mono_draw_string("**", 16,16, &sysfont);
			
		}
		if(count == 12){
			gfx_mono_draw_string("***", 16,16, &sysfont);
			
		}
		if(count == 16){
			gfx_mono_draw_string("****", 16,16, &sysfont);
			
		}
		if(count == 20){			
			gfx_mono_draw_string("    ", 16,16, &sysfont);	
			count = 0;
		}
		
		//gfx_mono_ssd1306_init();
		
		
		
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

	}
}
