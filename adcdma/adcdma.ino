//#include "wiring_private.h" // BUGBUG : This standard Arduino file not present?
#include "stm32l4_wiring_private.h"

#define PRREG(x) Serial.print(#x" 0x"); Serial.println(x,HEX) // Debug print HEX
#define qBlinkg() (digitalWrite(A3, !digitalRead(A3) ))

#define DAC_PIN A1
#define TIM6_USED 1 // undefine to use TIM2
#define ADC_PIN A2  // #define ADC_PIN A4
#define DMA_BUF_SIZE 64
#define FREQHZ 10000  // Timer drive ADC samples per second

volatile uint32_t ticks;
void timer_isr(void *context, uint32_t events) {
  ticks++;
}

static stm32l4_timer_t mytimer;

#define DMA_OPTIONS (DMA_OPTION_PERIPHERAL_TO_MEMORY | DMA_OPTION_MEMORY_DATA_SIZE_16  | DMA_OPTION_PERIPHERAL_DATA_SIZE_16 | DMA_OPTION_MEMORY_DATA_INCREMENT  \
                     | DMA_OPTION_MEMORY_DATA_INCREMENT | DMA_OPTION_EVENT_TRANSFER_DONE )
//                     | DMA_OPTION_CIRCULAR | DMA_OPTION_EVENT_TRANSFER_DONE | ADC_CFGR_CONT ) // Continuous : ADC_CFGR_CONT use DMA_OPTION_CIRCULAR

stm32l4_dma_t dma;

// BUGBUG from : \hardware\stm32l4\0.0.26\system\STM32L4xx\Source\stm32l4_adc.c
#define ADC_SAMPLE_TIME_47_5   4

uint16_t adcbuf[2][DMA_BUF_SIZE];   //  ADC DMA buffer
#define ADCBUF_CNT sizeof(adcbuf[0])/2
uint32_t t1, t2, tct, tch = 40000; // Test Timer values
volatile uint32_t ButtonHits = 1;
volatile uint32_t iiButtonHits = 1;
uint32_t DAC_write = 4060;

void initADC(int pin) {
  int channel = g_APinDescription[pin].adc_input;
  // configure
  stm32l4_system_periph_enable(SYSTEM_PERIPH_ADC);
  ADC1->CR &= ~ADC_CR_DEEPPWD;
  ADC1->CR |= ADC_CR_ADVREGEN;

  delayMicroseconds( 20000 );
  // ??? armv7m_clock_spin(20000);  // BUGBUG : WHAT DID THIS DO - It is no longer present? Assumed delay?

  ADC1->ISR = ADC_ISR_ADRDY;
  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY));
#ifdef TIM6_USED
  ADC1->CFGR = ADC_CFGR_OVRMOD | ADC_CFGR_EXTEN_0 | ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_2 |  ADC_CFGR_EXTSEL_0 | ADC_CFGR_DMAEN;    // configure:  select TIM6 TRGO rising external event
#else
  ADC1->CFGR = ADC_CFGR_OVRMOD | ADC_CFGR_EXTEN_0 | ADC_CFGR_EXTSEL_3 | ADC_CFGR_EXTSEL_1 |  ADC_CFGR_EXTSEL_0 | ADC_CFGR_DMAEN;  // configure:  select TIM2 TRGO rising external event
#endif

  stm32l4_gpio_pin_configure(g_APinDescription[pin].pin, (GPIO_PUPD_NONE | GPIO_MODE_ANALOG | GPIO_ANALOG_SWITCH));
  uint32_t adc_smp = ADC_SAMPLE_TIME_47_5;
  ADC1->SQR1 = (channel << 6);
  ADC1->SMPR1 = (channel < 10) ? (adc_smp << (channel * 3)) : 0;
  ADC1->SMPR2 = (channel >= 10) ? (adc_smp << ((channel * 3) - 30)) : 0;

#ifdef SHOW_DEBUG
  PRREG(ADC1->SQR1); PRREG(ADC1->SMPR1); PRREG(ADC1->ISR); PRREG(ADC1->CR); PRREG(ADC1->CFGR);
#endif
}

void disableADC() {
  ADC1->CR |= ADC_CR_ADDIS;
  while (ADC1->CR & ADC_CR_ADEN)   { }
  ADC1->CR &= ~ADC_CR_ADVREGEN;
  ADC1->CR |= ADC_CR_DEEPPWD;
  stm32l4_system_periph_disable(SYSTEM_PERIPH_ADC);
}

void readADC( bool bWait ) { // Don't wait with callback enabled - it never happens!
  ADC1->CR |= ADC_CR_ADSTART;
  stm32l4_dma_start(&dma, (uint32_t)adcbuf[0], (uint32_t) & (ADC1->DR),  ADCBUF_CNT, DMA_OPTIONS);
  while ( bWait && ! stm32l4_dma_done(&dma));
}

typedef void (*stm32l4_ADC_callback_t)(void *context, uint32_t events);

static void stm32l4_ADC_dma_callback(void *unused, uint32_t events)
{
  tch = micros() - tct;
  ADC1->CR |= ADC_CR_ADSTART;
  qBlinkg();
  // TODO - Swap start Buffer and copy out prior to pool 0 or 1
  // stm32l4_dma_start(&dma, (uint32_t)adcbuf[bBuff], (uint32_t) & (ADC1->DR),  sizeof(adcbuf) / 2, DMA_OPTIONS);
  stm32l4_dma_start(&dma, (uint32_t)adcbuf[0], (uint32_t) & (ADC1->DR),  ADCBUF_CNT, DMA_OPTIONS);
  tct = micros();
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  pinMode(A3, OUTPUT);
  pinMode(DAC_PIN, 4096);
  analogWriteResolution( 12 );
  analogWrite( DAC_PIN, DAC_write );
  pinMode(PIN_BUTTON, INPUT);
  attachInterrupt(PIN_BUTTON, isrButton, RISING);

  // init ADC
  analogRead(ADC_PIN);   // init ADC1
  initADC(ADC_PIN);

  // init DMA
  stm32l4_dma_create(&dma, DMA_CHANNEL_DMA1_CH1_ADC1, DMA_OPTION_PRIORITY_MEDIUM);
  stm32l4_dma_enable(&dma, (stm32l4_ADC_callback_t)stm32l4_ADC_dma_callback, NULL);
  stm32l4_dma_start(&dma, (uint32_t)adcbuf[0], (uint32_t) & (ADC1->DR),  ADCBUF_CNT, DMA_OPTIONS);

  // init timer
#ifdef TIM6_USED
  stm32l4_timer_create(&mytimer, TIMER_INSTANCE_TIM6, STM32L4_TONE_IRQ_PRIORITY, 0);
#else
  stm32l4_timer_create(&mytimer, TIMER_INSTANCE_TIM2, STM32L4_TONE_IRQ_PRIORITY, 0);
#endif

  uint32_t modulus = (stm32l4_timer_clock(&mytimer) / FREQHZ) ;
  uint32_t scale   = 1;
  while (modulus > 65536) {
    modulus /= 2;
    scale++;
  }
  stm32l4_timer_enable(&mytimer, scale - 1, modulus - 1, 0, timer_isr, NULL, TIMER_EVENT_PERIOD);

  Serial.print("\n Using ADC pin:");
  Serial.print(ADC_PIN);
  Serial.print(" tied to DAC pin:");
  Serial.print(DAC_PIN);
#ifdef TIM6_USED
  TIM6->CR2 = 0x20;
  Serial.print(" and TIM6 @");
#else
  TIM2->CR2 = 0x20;  // MMS TRGO update
  Serial.print(" and TIM2 @");
#endif

  char str[64];
  sprintf(str, "%d hz  scale %d modulus %d  clockhz %d", FREQHZ, scale, modulus, stm32l4_timer_clock(&mytimer));
  Serial.println(str);

  stm32l4_timer_start(&mytimer, false);

#ifdef SHOW_DEBUG
  PRREG(ADC1->SQR1); PRREG(ADC1->SMPR1); PRREG(ADC1->ISR); PRREG(ADC1->CR); PRREG(ADC1->CFGR);
#endif

  // Start DMA CALLBACK reads
  ADC1->CR |= ADC_CR_ADSTART;
  stm32l4_dma_start(&dma, (uint32_t)adcbuf[0], (uint32_t) & (ADC1->DR),  ADCBUF_CNT, DMA_OPTIONS);

  delayMicroseconds( tch ); // DEBUG - Pause for first DMA to get data
  for (int i = 0; i < 16; i++) {
    Serial.print(adcbuf[0][i]);
    Serial.print( ", " );
  }
  Serial.print("ADCBUF_CNT ="); Serial.println(ADCBUF_CNT);
  Serial.println(adcbuf[0][3]);
  memset(adcbuf, 7, sizeof(adcbuf));
}

void loop() {
  if ( iiButtonHits < 9 ) // Comment this IF to just RUN
  { // Cycle on startup
    ButtonHits++;
    DAC_write += 47;
    if ( DAC_write > 4060 ) DAC_write = 1;
    analogWrite( DAC_PIN, DAC_write );
  }
  if ( ButtonHits != iiButtonHits ) {
    delayMicroseconds( tch );
    Serial.print(ticks); Serial.println(" timer ticks " );
    uint32_t DAC_PIN_Eq;
    Serial.print(DAC_write); Serial.print(" = DAC_write\t\t");
    Serial.print("CallBack us "); Serial.println(tch);
    do {
      DAC_PIN_Eq = 0;
      for ( int ii = 0; ii < ADCBUF_CNT; ii++ ) {
        if ( ii )       Serial.print(", ");
        Serial.print(adcbuf[0][ii]);
        if ( DAC_write <=  adcbuf[0][ii] ) DAC_PIN_Eq++;
      }
      delayMicroseconds( tch );
      Serial.print(" :: DAC_PIN_Eq="); Serial.print(DAC_PIN_Eq);
      Serial.println();
    } while ( DAC_PIN_Eq < 3 );
    Serial.println("------------");
    iiButtonHits = ButtonHits;
  }
}

void isrButton()
{
  qBlinkg();
  if ( ButtonHits == iiButtonHits )
    ButtonHits++;
  DAC_write += 27;
  if ( DAC_write > 4060 ) DAC_write = 1;
  analogWrite( DAC_PIN, DAC_write );
}

