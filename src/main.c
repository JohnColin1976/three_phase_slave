#include "sam3xa.h"
#include "sin_lut_1024.h"
#include "init.h"
#include "uart.h"

// Конфигурация под сигнальный PB27 - для вывода на светодиод
// событий, в том числе и за счет различной последовательности
// морганий
#define TEST_PIO      PIOB
#define TEST_PIN      27u
#define TEST_MASK     (1u << TEST_PIN)

// Конфиг пинов SYNC_OUT (пример: PB26) + флаг SYNC_PIO
#define SYNC_IN_PIO      PIOB
#define SYNC_IN_PIN      26u
#define SYNC_IN_MASK     (1u << SYNC_IN_PIN)


// Настройка сигнала синуса
static volatile uint32_t phase = 0;
static volatile uint32_t phase_inc = 0;   // задаём частоту
static volatile uint16_t amp = 2047;      // 0..2047 (масштаб амплитуды)

// Настройка синхронизации
static volatile uint32_t sync_div = 100; // 100kHz / 100 = 1kHz
static volatile uint32_t sync_cnt = 0;
static volatile uint8_t g_sync_seen = 0;

// Сдвиги фаз на 120° и 240° в 32-битной фазе:
#define PHASE_120 0x55555555u
#define PHASE_240 0xAAAAAAAau


// Генерация синусоидальных значенийP
static inline uint16_t synth_u12(uint32_t ph) {
  // 1024 точки => 10 бит индекса из старших бит фазы
  uint32_t idx = ph >> 22; // 32 - 10
  int32_t s = (int32_t)sinLUT_1024[idx] - 2048;
  s = (s * (int32_t)amp) >> 11;
  return (uint16_t)(s + 2048);
}

static inline void dds_set_freq_hz(uint32_t f_hz) {
  // phase_inc = f_out * 2^32 / f_update
  // f_update = 100000 (у нас RC=420 на MCK/2=42MHz)
  const uint32_t f_update = 100000u;
  phase_inc = (uint32_t)(((uint64_t)f_hz << 32) / f_update);
}

void PIOB_Handler(void) __attribute__((used));
void PIOB_Handler(void) {
  PIOB->PIO_ISR;   // ACK! обязательно
  g_sync_seen = 1;
}

//Переключение PB27
static inline void toggle_pin(void) {
  if (TEST_PIO->PIO_ODSR & TEST_MASK) TEST_PIO->PIO_CODR = TEST_MASK;
  else                                TEST_PIO->PIO_SODR = TEST_MASK;
}


void TC0_Handler(void) __attribute__((used));
void TC0_Handler(void) {
  TcChannel *tc = &TC0->TC_CHANNEL[0];
  uint32_t sr = tc->TC_SR;                  // ACK
  if (sr & TC_SR_CPCS) {

    // SYNC применяется строго на границе дискретизации
    if (g_sync_seen) {
      g_sync_seen = 0;
      phase = 0;
    }

    phase += phase_inc;

    uint16_t vc = synth_u12(phase + PHASE_240);

    while ((DACC->DACC_ISR & DACC_ISR_TXRDY) == 0) {}
    DACC->DACC_CDR = (0u << 12) | (vc & 0x0FFF);
  }
}

static inline void dacc_write_ch(uint8_t ch, uint16_t v12) {
  while ((DACC->DACC_ISR & DACC_ISR_TXRDY) == 0) { }
  DACC->DACC_CDR = ((uint32_t)ch << 12) | (v12 & 0x0FFF);
}

static volatile uint32_t g_ms_ticks = 0u;

void SysTick_Handler(void) __attribute__((used));
void SysTick_Handler(void) {
  g_ms_ticks++;
}

static inline void delay_ms(uint32_t ms) {
  uint32_t start = g_ms_ticks;
  while ((uint32_t)(g_ms_ticks - start) < ms) {
    __NOP();
  }
}



int main(void) {
  // 1) снять защиту PMC и включить тактирование
  PMC->PMC_WPMR = 0x504D4300;                  // WPKEY="PMC", WPEN=0
  PMC->PMC_PCER0 =  (1u << ID_PIOB) |   // Включение тактирования для PIOB
                    (1u << ID_TC0) |    // Включение тактирования для TC0
                    (1u << ID_USART0) | // Включение тактирования для USART0
                    (1u << ID_USART1);  // Включение тактирования для USART1

  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000u);

  uart_init();

  sync_in_init();

  dacc_init_slave();

  amp = 1800;          // амплитуда (подбери)
  dds_set_freq_hz(50); // старт: 50 Гц для проверки

  // 2) gpio
  gpio_init_out();

  // 3) снять защиту TC0
  TC0->TC_WPMR = 0x54430000;                   // WPKEY="TC", WPEN=0

  TcChannel *tc = &TC0->TC_CHANNEL[0];

  // 4) стоп + сброс IRQ/статуса
  tc->TC_CCR = TC_CCR_CLKDIS;
  tc->TC_IDR = 0xFFFFFFFFu;
  (void)tc->TC_SR;

  // 5) Waveform, UP to RC, clock = MCK/2
  tc->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |
               TC_CMR_WAVE |
               TC_CMR_WAVSEL_UP_RC;

  // 84MHz/2 = 42MHz; 42MHz/420 = 100kHz
  tc->TC_RC  = 420;

  /* 7) Разрешить IRQ по RC compare */
  tc->TC_IER = TC_IER_CPCS;
  

  /* 8) NVIC */
  NVIC_DisableIRQ(TC0_IRQn);
  NVIC_ClearPendingIRQ(TC0_IRQn);
  NVIC_SetPriority(TC0_IRQn, 0);   // высокий приоритет
  NVIC_EnableIRQ(TC0_IRQn);



  // 6) старт
  tc->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

  //volatile uint32_t cv_test;
  while (1) {
    //cv_test = TC0->TC_CHANNEL[0].TC_CV;
    //__WFI();
    //uart0_send_text("UART0 TEST\r\n");
    //delay_ms(500);

    if (uart0_poll_com_n_gen()) {
      dds_set_freq_hz(g_params_in.com_n_gen);
    }
  }
}
