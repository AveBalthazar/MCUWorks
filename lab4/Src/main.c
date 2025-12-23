// лаб 4, отключил намеренно внешний источник тактирования и PLL потому что на моем мк нестабильный HSE

#include <stdint.h>
#include <stm32f446xx.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#define V_REF           3.3f  // V_DDA
#define VOLTAGE_START   1.65f
#define APB1_FREQ       25000000U
#define APB2_FREQ       25000000U
#define TIM6_UF_FREQ    10000       // frequency of update interrupt of TIM in hz
#define BAUDRATE        1000000U
#define ADC_IN_NUMBER   1           // adc channel number (PA1, ADC123_IN1)

#define PLLM            4
#define PLLN            100

#define PACKET_SIZE             3
#define PACKET_HEADER           0xAA
#define SERVICE_BYTE_NUMBER     1

volatile bool dacTimerUpdate = false;
volatile bool adcConverted = false;
volatile bool txBusy = false;
volatile bool adcFlag = false;
unsigned short packet_length;

const uint16_t TIM6_PSC = (APB1_FREQ * 2 / 1000000U) - 1;  // частота счетчика таймера 1мгц (прескейлер на APB1 != 1)
const uint16_t TIM6_ARR = (APB1_FREQ * 2 / TIM6_PSC) / TIM6_UF_FREQ;  // для обеспечения частоты флага UF указанной в TIM_UF_FREQ

uint16_t n = 0;
uint16_t dots_on_period;
float amplitude = 2.0f;
float nu = 15.0f;

float voltage;
float omega;
float dt;
float period;

uint16_t adcBuffer = 0;
uint8_t txBuffer[PACKET_SIZE];

void fill_sin_array(float_t *sin_dots, uint32_t dots_quantity) {
    for (uint32_t i = 0; i < dots_quantity; i++) {
        sin_dots[i] = (amplitude / 2.0f) * sinf(omega * (float)i / TIM6_UF_FREQ);
    }
}

static void enableFPU(void) {
    SCB->CPACR |= (3UL << 10 * 2) | (3UL << 11 * 2);  // разобрать как работает FPU
}

static void configureFLASH(void) {
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_1WS;
}

static void set_signal_parameters(void) {
    omega = 2 * M_PI * nu;
    dots_on_period = (uint16_t)TIM6_UF_FREQ/nu;
}

static void send_adc_data_to_txbuffer(uint16_t adc_value) {
    if (txBusy) return;
    txBuffer[0] = PACKET_HEADER;
    txBuffer[1] = (adc_value >> 8);
    txBuffer[2] = (adc_value & 0xFF);

    txBusy = true;
    DMA1_Stream6->NDTR = 3;  // amount of bytes to transfer
    DMA1_Stream6->M0AR = (uint32_t)txBuffer;
    DMA1_Stream6->CR |= DMA_SxCR_EN;  // stream 6 enable
}

static void configureRCC(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    RCC->CFGR &= ~(RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE1_0);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 freq = HCLK / 2 (25)
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;  // APB2 freq = HCLK / 2 (25)

    // RCC->CR |= RCC_CR_HSEBYP;  // hse gets ticks from bypass

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLSRC_Msk);
    RCC->PLLCFGR |= (PLLM << RCC_PLLCFGR_PLLM_Pos);  // VCO_clock = HSE(16) / PLLM
    RCC->PLLCFGR |= (PLLN << RCC_PLLCFGR_PLLN_Pos); // VCO_clock = HSE(16) / PLLM * PLLN
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_0;  // PLL_output = VCO_clock / PLLP (50)
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
    
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_GPIOCEN;
}

// ADC срабатывает по триггеру когда выставляется UF из TIM2
static void configureADC(void) {
    ADC1->CR2 |= ADC_CR2_ADON;  // ADC enable
    while (!(ADC1->CR2 & ADC_CR2_ADON)) {}
    ADC1->CR2 &= ~ADC_CR2_EXTEN_Msk; 
    ADC1->CR2 |= ADC_CR2_EXTEN_0;  // rising edge
    ADC1->CR2 &= ~ADC_CR2_EXTSEL_Msk;
    ADC1->CR2 |= ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;  // timer 2 TRGO event
    
    ADC1->SQR3 &= ~ADC_SQR3_SQ1_Msk;
    ADC1->SQR3 |= ADC_IN_NUMBER;
    ADC1->SQR1 &= ~ADC_SQR1_L_Msk;  // 1 conversion by default
    ADC1->CR1 &= ~ADC_CR1_RES_Msk;  // 12-bit resolution
    ADC1->SMPR2 &= ~ADC_SMPR2_SMP0_Msk;
    ADC1->SMPR2 |= (ADC_SMPR2_SMP0_0);  // reading 12-bit data needs at least 15 ADCCLK cycles, chosen 15
    ADC1->CR2 &= ~ADC_CR2_CONT;  // single conversion mode
    ADC1->CR1 |= ADC_CR1_EOCIE;
}

static void configureDAC(void) {
    DAC1->CR |= DAC_CR_EN1;
    while ((DAC->CR & DAC_CR_EN1_Msk) != DAC_CR_EN1);
    DAC1->DHR12R1 = 2048;  // half of maximum voltage
}

static void configureGPIO(void) {
    GPIOA->MODER &= ~(GPIO_MODER_MODE1_Msk | GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk | GPIO_MODER_MODE4_Msk);
    GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);  // PA2, PA3 - rx, tx (AF mode required)
    GPIOA->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE4_0);  // PA4 - DAC1 out (analog mode required)
    GPIOA->MODER |= (GPIO_MODER_MODE1_1 | GPIO_MODER_MODE1_0);  // PA1 - ADC123_IN1 (analog mode required)
}

static void configureUSART(void) {
    USART2->BRR = APB1_FREQ/BAUDRATE;  // buadrate is 1kk
    USART2->CR1 |= USART_CR1_TE;
    // USART2->CR1 &= ~USART_CR1_TXEIE;
    // USART2->CR1 &= ~USART_CR1_OVER8;  // Режим передискретизации в 16 раз (по умолчанию) (надо включить x8?)
    USART2->CR1 &= ~(USART_CR1_M_Msk | USART_CR1_PCE_Msk);  // 8 data bits, parity control disabled
    USART2->CR3 |= USART_CR3_DMAT;
    USART2->CR2 &= ~(USART_CR2_STOP_Msk);  // 1 stop bit
    USART2->CR1 |= USART_CR1_UE;
    GPIOA->AFR[0] &= ~((0xF << (4*2)) | (0xF << (4*3)));
    GPIOA->AFR[0] |= (7 << (4*2)) | (7 << (4*3)); // AF7 = USART2 TX/RX
}

static void configureTIM6(void) {
    TIM6->PSC = TIM6_PSC;  // CK_INT = APB1_FREQ * 2 = 50 MHz => CK_PSC = 1 MHz
    TIM6->ARR = TIM6_ARR;  // update event every 0.1ms
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
}

static void configureTIM2(void) {
    TIM2->PSC = TIM6_PSC+1;  // CK_INT = APB1_FREQ * 2 = 50 MHz => CK_PSC = 1 MHz
    TIM2->ARR = TIM6_ARR+1;  // update event every 0.1ms
    TIM2->CR2 &= ~TIM_CR2_MMS_Msk;
    TIM2->CR2 |= TIM_CR2_MMS_1;  // The update event is selected as trigger output
    TIM2->CR1 |= TIM_CR1_CEN;
}

void configureDMA(void) {
    DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
    DMA1_Stream6->CR |= 4 << DMA_SxCR_CHSEL_Pos;  // 4th external channel
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;
    DMA1_Stream6->CR |= DMA_SxCR_DIR_0;  // data transfer direction: memory-to-peripheral
    DMA1_Stream6->CR |= DMA_SxCR_MINC;
    DMA1_Stream6->CR |= DMA_SxCR_TCIE;
    DMA1_Stream6->CR &= ~(DMA_SxCR_PSIZE_Msk);  // then MSIZE is equal to PSIZE = 8 bit
}

static void enableIRQ(void) {
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    // NVIC_EnableIRQ(USART2_IRQn);
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

int main(void) {
    configureFLASH();
    configureRCC();
    enableFPU();
    set_signal_parameters();
    configureGPIO();
    configureDAC();
    configureADC();
    configureDMA();
    configureUSART();
    configureTIM6();
    configureTIM2();

    enableIRQ();
    
    float_t sin_dots[dots_on_period];
    fill_sin_array(sin_dots, dots_on_period);

    // ADC1->CR2 |= ADC_CR2_SWSTART;
    while (1) {
        if (dacTimerUpdate) {
            voltage = VOLTAGE_START + (sin_dots[n]);
            DAC->DHR12R1 = (uint32_t)(voltage / V_REF * 4095.0f);
            dacTimerUpdate = false;
        } if (adcFlag) {
            send_adc_data_to_txbuffer(adcBuffer);
            adcFlag = false;
        }
    }
}

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR &= ~TIM_SR_UIF_Msk;
        if (n++ == (uint32_t)dots_on_period-1) {n = 0;}
        dacTimerUpdate = true;
    }
}

void ADC_IRQHandler(void) {
    if (ADC1->SR & ADC_SR_EOC) {
        adcBuffer = (uint16_t)ADC1->DR;  // flag is cleared by reading from ADC1_DR
        adcFlag = true;
    }
}

void DMA1_Stream6_IRQHandler(void) {
    if ((DMA1->HISR & DMA_HISR_TCIF6)) {  // передача данных в usart->dr для tx завершена
        txBusy = false;
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
    }
}
