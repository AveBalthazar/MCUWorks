// изменено количество sampling циклов в adc, может сломать всё (было 84)
// нужно ли ставить TIM2->CCMR2 |= TIM_CCMR2_OC3PE? (с этим CCR будет обновляться лишь на следующий UIF а не сразу)
// нужно ли ставить TIM_EGR_UG в таймерах? и так сразу же обновятся значения. где-то помимо инициализации как будто есть смысл, а оттуда можно убрать 
// происходит ли проверка на достижение двигателем крайних положений и отключение? если нет то deadzone на самом деле не deadzone а как то ещё называется 

#include "stm32f446xx.h"
#include <stdbool.h>

// dma put first part of data as internal potentiometer, second as an external
static volatile uint16_t adcBuffer[2];  

#define TX_MAX_SIZE     64U
#define APB1_FREQ       25000000U

// значение в градусах, при достижении ошибкой которого двигатель останавливается
#define DEADZONE_DEG    1.0f

// коэффициент умножения, задающий точность. умножение float на него даёт целочисленное число с сохранёнными цифрами после запятой, при обработке телеметрических данных можно обратно получить исходное float
#define GAIN_COEFF      100U

#define TIM2_PSC        ((APB1_FREQ / 1000000U) - 1)
#define TIM2_ARR        (1000 - 1)
#define TIM3_PSC        ((APB1_FREQ / 1000000U) - 1)
#define TIM3_ARR        (1000 - 1)

// номер входа ADC, отвечающий за внутренний потенциометр (PC0)
#define INT_POT_ADC_NUM 10U

// номер входа ADC, отвечающий за внешний потенциометр (PB0)
#define EXT_POT_ADC_NUM 8U

#define PLLM            4U
#define PLLN            100U

#define BAUDRATE        115200U

static volatile uint8_t txBuffer[TX_MAX_SIZE];
static volatile uint8_t txLength = 0;
static volatile uint8_t txPos = 0;
static volatile uint8_t txBusy = 0;
static volatile bool tim3Flag = false;

// выводит ШИМ-сигнал на AF1 в мультиплексоре альтернативных функций (через GPIO->MODER PB10 (вход EN2) получает этот сигнал)
static void configureTIM2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1 &= ~TIM_CR1_CEN;

    TIM2->PSC = TIM2_PSC;
    TIM2->ARR = TIM2_ARR;

    TIM2->CCMR2 &= ~(TIM_CCMR2_CC3S_Msk | TIM_CCMR2_OC3M_Msk);
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);  // Output compare mode - PWM 1 (1 when CNT < CCR if polarity is direct)
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE;  // Output compare 3 preload enable (CCR3 changes only when UIF occurs)

    TIM2->CCER |= TIM_CCER_CC3E;  // enable the output (voltage to pin)
    TIM2->CCER &= ~TIM_CCER_CC3P_Msk;  // direct polarity
    TIM2->CCR3 &= ~(TIM_CCR3_CCR3_Msk);  // stop the engine

    TIM2->EGR |= TIM_EGR_UG;  // может быть удалить? и так же поставится
    TIM2->CR1 |= TIM_CR1_CEN;
}


static void configureRCC(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    RCC->CFGR &= ~(RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE1_0);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 freq = HCLK / 2 (25)
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;  // APB2 freq = HCLK / 2 (25)

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLSRC_Msk);
    RCC->PLLCFGR |= (PLLM << RCC_PLLCFGR_PLLM_Pos);  // VCO_clock = HSE(16) / PLLM
    RCC->PLLCFGR |= (PLLN << RCC_PLLCFGR_PLLN_Pos); // VCO_clock = HSE(16) / PLLM * PLLN
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_0;  // PLL_output = VCO_clock / PLLP (50)
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
}

static void configureFLASH(void) {
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_1WS;
}

static void configureADC1(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; (void)RCC->APB2ENR;

    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;

    ADC1->CR1 = ADC_CR1_SCAN;  // all members of the sequence are processed, not one for one activation

    ADC1->SQR1 = ADC_SQR1_L_0;  // L = 1, the length of the sequence is 2 (2 members)
    
    ADC1->SQR3 = (EXT_POT_ADC_NUM) | (INT_POT_ADC_NUM << 5);  // 1st conversion in sequence is 8th channel (PB0, external potentiometer), 2nd is 10th (PC0, internal potentiometer)

    ADC1->SMPR2 &= ~ADC_SMPR2_SMP8_Msk;
    ADC1->SMPR2 |= ADC_SMPR2_SMP8_0;  // 15 cycles
    ADC1->SMPR1 &= ~ADC_SMPR1_SMP10_Msk;
    ADC1->SMPR1 |= ADC_SMPR1_SMP10_0;  // 15 cycles

    ADC1->CR2 = 0;
    ADC1->CR2 = 
            ADC_CR2_DMA 
            | ADC_CR2_DDS  // DMA activates only when all members of the sequence are converted (not after each)
            | ADC_CR2_CONT;

    ADC1->CR2 |= ADC_CR2_ADON;
}

static void configureGPIO(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                   RCC_AHB1ENR_GPIOBEN |
                   RCC_AHB1ENR_GPIOCEN;

    // PA2 - AF (usart rx), PA3 - AF (usart tx), PA8 - output (motor DIR2)
    GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk | GPIO_MODER_MODER8_Msk);
    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER8_0;
    GPIOA->AFR[0] &= ((0xF << (2*4)) | (0xF << (3*4)) | (3U << (8*2)));
    GPIOA->AFR[0] |= (7U << (2*4)) | (7U << (3*4));  // AF7 = USART2 TX/RX
    GPIOA->PUPDR = (GPIOA->PUPDR & ~((3U << (2*2)) | (3U << (3*2))))
                 |  (1U << (3*2));

    // PB0 - analog (ext potentiometer), PB10 - AF (motor EN2, PWM)
    GPIOB->MODER &= ~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE10_Msk);
    GPIOB->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE0_0 | GPIO_MODER_MODE10_1;
    GPIOB->AFR[1] &= ~(0xF << (2*4));
    GPIOB->AFR[1] |= (1U << (2*4));  // PB10, AF1 = TIM2_CH3
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk | GPIO_PUPDR_PUPD10_Msk);  // pull-down

    // PC0 - analog (int potentiometer)
    GPIOC->MODER &= ~GPIO_MODER_MODE0_Msk; 
    GPIOC->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;  // pull-down
}

static void configureDMA2(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}
    
    DMA2_Stream0->CR &= ~(0x7 << 25);  // channel 0 selected (ADC1)

    DMA2_Stream0->CR =
            DMA_SxCR_CIRC
            | DMA_SxCR_MINC
            | DMA_SxCR_MSIZE_0  // 16-bit 
            | DMA_SxCR_PSIZE_0;

    DMA2_Stream0->FCR = 0;

    DMA2_Stream0->NDTR = 2;
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adcBuffer;

    DMA2_Stream0->CR |= DMA_SxCR_EN;  // DMA2S0 is activated from ADC1 trigger
}

static void configureDMA1(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; 
    DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
    DMA1_Stream6->CR |= 4 << DMA_SxCR_CHSEL_Pos;  // USART2_TX
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;
    DMA1_Stream6->CR |= DMA_SxCR_DIR_0;  // memory-to-peripheral
    DMA1_Stream6->CR |= DMA_SxCR_MINC;
    DMA1_Stream6->CR |= DMA_SxCR_TCIE;
    DMA1_Stream6->CR &= ~(DMA_SxCR_PSIZE_Msk);  // then MSIZE is equal to PSIZE = 8 bit
}

static void configureUSART2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->CR1 |= USART_CR1_IDLEIE;  // разрешаем прерывания для состояния простоя
    USART2->CR3 |= USART_CR3_DMAT;  // DMA enable transmitter
    USART2->BRR = (APB1_FREQ/BAUDRATE);
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

static void configureTIM3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;

    TIM3->CR1 &= ~TIM_CR1_CEN;

    TIM3->PSC = TIM3_PSC;
    TIM3->ARR = TIM3_ARR;

    TIM3->EGR |= TIM_EGR_UG;
    TIM3->SR  &= ~TIM_SR_UIF;
    TIM3->DIER |= TIM_DIER_UIE;

    TIM3->CR1 |= TIM_CR1_CEN;
}

static void configureNVIC(void) {
    // NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_SetPriority(TIM3_IRQn, 1);
    NVIC_SetPriority(DMA1_Stream6_IRQn, 0);
    
    // NVIC_EnableIRQ(USART2_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

#define K_P             1.0f

static void control_step(void) {
    uint16_t pos = (270.0f * (float)adcBuffer[0]) / 4095.0f;
    uint16_t ref = (270.0f * (float)adcBuffer[1]) / 4095.0f;
    
    float err = ref - pos;

    if (err > -DEADZONE_DEG && err < DEADZONE_DEG) {
        TIM2->CCR3 = 0;  // stop motor
        return;
    }

    if (err > 0.0f) {
        GPIOA->BSRR = GPIO_BSRR_BS_8;  // direction forward
    } else {
        GPIOA->BSRR = GPIO_BSRR_BR_8;  // direction reverse
        err = -err;
    }

    float u = K_P * err;

    TIM2->CCR3 = (uint32_t)(u * (float)TIM2->ARR);
}


static inline void send_char_to_tx(char c) {
    if (txLength < TX_MAX_SIZE) {
        txBuffer[txLength++] = (uint8_t)c;
    }
}

static inline void send_byte_to_tx(uint8_t b) {
    if (txLength < TX_MAX_SIZE) {
        txBuffer[txLength++] = b;
    }
}

static void send_int_to_tx(int16_t v) {
    char tmp[6];
    int i = 0;

    if (v < 0) {
        send_char_to_tx('-');
        v = -v;
    }

    if (v == 0) {
        send_char_to_tx('0');
        return;
    }

    while (v > 0) {  // digits to char
        tmp[i++] = '0' + (v % 10); 
        v /= 10;
    }
    while (i--) {
        send_char_to_tx(tmp[i]);
    }
}

// Умножает флотовое число на GAIN_COEFF (некоторую степень 10), возвращает целочисленное значение. Используется для сохранения знаков после запятой там, где приемлемы только целочисленные значения (к примеру, передача по UART)
static int16_t float_gain(float val) {
    return (int16_t)(val * GAIN_COEFF);
}

static void send_telemetry(void) {
    float pos = (270.0f * (float)adcBuffer[1]) / 4095.0f;
    float ref = (270.0f * (float)adcBuffer[0]) / 4095.0f;;
    float error = pos - ref;
    float controlValue = 100.0f * (float)TIM2->CCR3 / (float)TIM2->ARR;

    int16_t posTx = float_gain(pos);
    int16_t refTx = float_gain(ref);
    int16_t errTx = float_gain(error);
    int16_t controlValTx = float_gain(controlValue);

    txLength = 0;
    txBusy = 1;
    txPos = 0;

    // send_byte_to_tx(0xFF);  // start byte

    send_int_to_tx(posTx);
    send_char_to_tx(' ');

    send_int_to_tx(refTx);
    send_char_to_tx(' ');

    send_int_to_tx(controlValTx);
    send_char_to_tx(' ');

    send_int_to_tx(errTx);

    send_char_to_tx('\r');
    send_char_to_tx('\n');

    
    DMA1_Stream6->NDTR = txLength;  // amount of bytes to transfer
    DMA1_Stream6->M0AR = (uint32_t)txBuffer;  // address to buffer from which the data will be taken
    DMA1_Stream6->CR |= 0x1;  // stream 6 enable
}


int main(void) {
    configureFLASH();
    SCB->CPACR |= (3UL << (10*2)) | (3UL << (11*2));  // fpu
    configureRCC();
    configureGPIO();
    configureTIM2();
    configureADC1();
    configureDMA1();
    configureDMA2();
    configureUSART2();
    configureTIM3();
    configureNVIC();

    ADC1->CR2 |= ADC_CR2_SWSTART;

    while (1) {
        if (tim3Flag) {
            control_step();
            if (!txBusy) {
                send_telemetry();
            }
        }
    }
}

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        tim3Flag = true;
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

void DMA1_Stream6_IRQHandler(void) {
    if ((DMA1->HISR & (0x1 << 21))) {  // флаг TC на 6 потоке
        txBusy = false;
        DMA1->HIFCR |= (0x1 << 21);
    }
}
