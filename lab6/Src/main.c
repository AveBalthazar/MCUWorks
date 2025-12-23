// неизвестно разрешение энкодеров (CPR, PPR), из-за этого не рассчитать максимально возможную скорость вращения ДПТ, и не проверить с максимальной ли он скоростью вращается (разве что по подаваемому через ШИМ напряжению)
// узнать, какие реально MAX_COUNTS_ON_SPIN, PPR_INT, PPR_EXT
// в control_step при проверке направления вращения (знак duty) мог ошибиться, проверить
// могут ли таймеры стартовать с ненулевого значениея? стоит ли явно сбрасывать их CNT при инициализации?

#include "stm32f446xx.h"
#include <stdbool.h>
#include <stdint.h>

#define HSE_SPEED                   12U                 /* частота внешнего источника тактирования, используется для расчёта коэффициентов PLL */

#define TX_MAX_SIZE                 64U
#define APB1_FREQ                   25000000U

// #define DEADZONE_DEG                3.0f                /*!< значение в градусах, при достижении ошибкой которого двигатель останавливается */
#define DESIRED_UART_ACCURACY       0.001f                /*!< желаемая точность для отправки по uart, 0.1 соответствует 1 знаку после запятой, 0.001 - трём*/

#define abs(x) ((x) < 0 ? -(x) : (x))

// желаемые частоты срабатывания UIF таймеров в гц
#define TIM6_FREQ                   200U
#define TIM5_FREQ                   200U
#define TIM4_FREQ                   1000U
#define TIM3_FREQ                   1000U
#define TIM2_FREQ                   1000U

#define TIM_PSC(PERIPH_FREQ, DESIRED_TIM_PSC_FREQ) ((PERIPH_FREQ/DESIRED_TIM_PSC_FREQ) - 1);
#define TIM_ARR(PERIPH_FREQ, TIM_PSC, DESIRED_TIM_FREQ) ((PERIPH_FREQ/(TIM_PSC + 1))/DESIRED_TIM_FREQ);

#define INT_POT_ADC_NUM             10U                 /* номер входа ADC, отвечающий за внутренний потенциометр (PC0)*/
#define EXT_POT_ADC_NUM             8U                  /* номер входа ADC, отвечающий за внешний потенциометр (PB0)*/

#define PLLM                        (HSE_SPEED / 2U)    /* после PLLM частота VCO input frequency должна равняться 2 MHz */
#define PLLN                        100U

#define BAUDRATE                    115200U

volatile int32_t prev_cnt_int = 0;                      // значение счетчика таймера на предыдущем вызове функции подсчета скорости
volatile int32_t prev_cnt_ext = 0;                      // значение счетчика таймера на предыдущем вызове функции
volatile int32_t current_motor_speed = 0;               // текущая скорость мотора (разность между двумя последними замерами TIM_CNT внутреннего энкодера)
static volatile uint8_t txBuffer[TX_MAX_SIZE];
static volatile uint8_t txLength = 0;
static volatile uint8_t txPos = 0;
static volatile uint8_t txBusy = 0;
static volatile bool telemetryFlag = false;             // флаг сработал значит пора отправлять телеметрию
static volatile bool readMotorSpeedFLag = false;        // флаг сработал значит пора читать скорость вращения дпт
static float target_speed_percentage = 0.0f;
static float current_speed_percentage = 0.0f;
static float error = 0.0f;
static volatile int32_t target_pulses = 0U;

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

static void configureGPIO(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                   RCC_AHB1ENR_GPIOBEN;

    // PA0, PA1 - AF TIM2 (int encoder out), PA2 - AF (usart rx), PA3 - AF (usart tx)
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk | GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
    GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
    GPIOA->AFR[0] &= ~((0xF << (0*4)) | (0xF << (1*4)) | (0xF << (2*4)) | (0xF << (3*4)));
    GPIOA->AFR[0] |= (7U << (2*4)) | (7U << (3*4));  // AF7 = USART2 TX/RX
    GPIOA->AFR[0] |= (1U << (0*4)) | (1U << (1*4));  // AF1 = TIM2 (CH1 & CH2)
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2_Msk | GPIO_PUPDR_PUPD3_Msk | GPIO_PUPDR_PUPD0_Msk | GPIO_PUPDR_PUPD1_Msk);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0;  // pull-up для энкодера 

    // PB4 - AF (motor EN1, PWM), PB5 - output (motor DIR1), PB6, PB7 - AF TIM4 (ext encoder out)
    GPIOB->MODER &= ~(GPIO_MODER_MODE4_Msk | GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE6_Msk);
    GPIOB->MODER |= GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;
    GPIOB->AFR[0] &= ~((0xF << (4*4)) | (0xF << (6*4)) | (0xF << (7*4)));
    GPIOB->AFR[0] |= (2U << (4*4));  // PB4, AF2 = TIM3_CH1
    GPIOB->AFR[0] |= (2U << (6*4)) | (2U << (7*4));  // AF2 = TIM4 (CH1 & CH2)
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk | GPIO_PUPDR_PUPD4_Msk | GPIO_PUPDR_PUPD6_Msk | GPIO_PUPDR_PUPD7_Msk);
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0;  // pull-up для энкодера
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

    USART2->CR1 |= USART_CR1_IDLEIE;
    USART2->CR3 |= USART_CR3_DMAT;  // DMA enable transmitter
    USART2->BRR = (APB1_FREQ/BAUDRATE);
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

// таймер для определения скорости вращения ДПТ
static void configureTIM6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    uint32_t TIM6_PSC = TIM_PSC(APB1_FREQ*2, 1000000U);
    TIM6->PSC = TIM6_PSC;
    TIM6->ARR = TIM_ARR(APB1_FREQ*2, TIM6_PSC, TIM6_FREQ);
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
}

// таймер для отправки телеметрии
static void configureTIM5(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    (void)RCC->APB1ENR;

    TIM5->CR1 &= ~TIM_CR1_CEN;

    uint32_t TIM5_PSC = TIM_PSC(APB1_FREQ*2, 1000000U);
    TIM5->PSC = TIM5_PSC;
    TIM5->ARR = TIM_ARR(APB1_FREQ*2, TIM5_PSC, TIM5_FREQ);

    TIM5->EGR |= TIM_EGR_UG;
    TIM5->SR  &= ~TIM_SR_UIF;
    TIM5->DIER |= TIM_DIER_UIE;

    TIM5->CR1 |= TIM_CR1_CEN;
}

// таймер 16-bit, каналы которого подключены к внешнему энкодеру
void configureTIM4(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = 0;
    TIM4->ARR = 0xFFFF;
    TIM4->EGR |= TIM_EGR_UG;
    TIM4->SMCR &= ~TIM_SMCR_SMS_Msk;
    TIM4->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;  // считаем и А, и В (х4)
    TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;  // входы TI1 (TIM4_CH1) и TI2 (TIM4_CH2)
    TIM4->CCMR1 &= ~(TIM_CCMR1_IC1F_Msk | TIM_CCMR1_IC2F_Msk);
    TIM4->CCMR1 |= (0x3 << TIM_CCMR1_IC1F_Pos) | (0x3 << TIM_CCMR1_IC2F_Pos);  // фильтруем
    TIM4->CR1 |= TIM_CR1_CEN;
}

// выводит ШИМ-сигнал на AF1 в мультиплексоре альтернативных функций, PB4 (вход EN1) получает этот сигнал
static void configureTIM3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->CR1 &= ~TIM_CR1_CEN;
    uint32_t TIM3_PSC = TIM_PSC(APB1_FREQ*2, 1000000U);
    TIM3->PSC = TIM3_PSC;
    TIM3->ARR = TIM_ARR(APB1_FREQ*2, TIM3_PSC, TIM3_FREQ);

    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk);
    TIM3->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);  // Output compare mode - PWM 1 (1 when CNT < CCR if polarity is direct)
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;  // Output compare 1 preload enable (CCR1 changes only when UIF occurs)

    TIM3->CCER |= TIM_CCER_CC1E;  // enable the output (voltage to pin)
    TIM3->CCER &= ~TIM_CCER_CC1P_Msk;  // direct polarity
    TIM3->CCR1 &= ~(TIM_CCR1_CCR1_Msk);  // stop the engine

    TIM3->EGR |= TIM_EGR_UG;  // может быть удалить? и так же поставится
    TIM3->CR1 |= TIM_CR1_CEN;
}

// таймер 32-bit, каналы которого подключены к внутреннему энкодеру
void configureTIM2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->SMCR &= ~TIM_SMCR_SMS_Msk;
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;  // encoder mode 2, считаем и А, и В (х4)
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;  // входы TI1 (TIM2_CH1) и TI2 (TIM2_CH2)
    TIM2->CCMR1 &= ~(TIM_CCMR1_IC1F_Msk | TIM_CCMR1_IC2F_Msk);
    TIM2->CCMR1 |= (0x3 << TIM_CCMR1_IC1F_Pos) | (0x3 << TIM_CCMR1_IC2F_Pos);  // фильтруем
    TIM2->CR1 |= TIM_CR1_CEN;
}


static void configureNVIC(void) {
    NVIC_SetPriority(TIM6_DAC_IRQn, 2);
    NVIC_SetPriority(TIM5_IRQn, 1);
    NVIC_SetPriority(DMA1_Stream6_IRQn, 0);
    
    NVIC_EnableIRQ(TIM5_IRQn);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

// функция для определения скорости вращения вала ДПТ (количество счётов которое подсчитал таймер с предыдущего значения)
static int32_t get_motor_speed(void) {
    int32_t current_cnt = (int32_t)TIM2->CNT;
    TIM2->CNT = 0;
    return current_cnt;
}

// количество импульсов на канал энкодера
#define PPR_INT 100U

// кол-во рисок которые энкодер должен пройти чтобы изменить напряжение на валу ДПТ от 0 до V_DDA или от 0 до -V_DDA
#define PPR_EXT 10U
#define CPR_EXT 40

// кол-во отсчётов (не рисок) появляющееся на счетчике таймера при максимальном значении напряжения на ДПТ за период срабатывания TIM6 (5 мс)
#define MAX_COUNTS_ON_SPIN           180U

#define MAX_TARGET_PULSES            40

// возвращает количество счётов таймера внешнего энкодера, которое было просчитано с предыдущего вызова функции
static int16_t get_ext_encoder_value(void) {
    int16_t cnt = (int16_t)TIM4->CNT;
    TIM4->CNT = 0;
    return cnt;
}

#define K_P            1.0f

static void control_step(void) {
    int16_t delta = get_ext_encoder_value();
    current_motor_speed = get_motor_speed();
    target_pulses += delta;  // поворот ручки внешнего энкодера в процентах от того, которое дало бы 100% прирост скорости
    if (target_pulses > MAX_TARGET_PULSES) {
        target_pulses = MAX_TARGET_PULSES;
    } else if (target_pulses < -MAX_TARGET_PULSES) {
        target_pulses = -MAX_TARGET_PULSES;
    }
    
    target_speed_percentage = (float)target_pulses / MAX_TARGET_PULSES;
    current_speed_percentage = (float)current_motor_speed / MAX_COUNTS_ON_SPIN;
    
    error = target_speed_percentage - current_speed_percentage;
    float duty = error * K_P;

    if (duty > 0.0f) {
        GPIOB->BSRR = GPIO_BSRR_BR_5;  // direction forward
    } else {
        GPIOB->BSRR = GPIO_BSRR_BS_5;  // direction reverse
        duty = -duty;
    }
    
    if (duty > 1.0f) duty = 1.0f;

    if (current_speed_percentage < 0.01f && current_speed_percentage > -0.01f && target_speed_percentage == 0) duty = 0;  // deadzone

    TIM3->CCR1 = (uint32_t)(duty * (float)(TIM3->ARR + 1U));
}


static inline void send_char_to_tx(char c) {
    if (txLength < TX_MAX_SIZE) {
        txBuffer[txLength++] = (uint8_t)c;
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


static void send_telemetry(void) {
    int16_t target_int = (int16_t)((float)target_pulses / (DESIRED_UART_ACCURACY * MAX_TARGET_PULSES * 10)) * (MAX_COUNTS_ON_SPIN / 4);
    int16_t actual_int = (int16_t)((float)current_speed_percentage / DESIRED_UART_ACCURACY) * (MAX_COUNTS_ON_SPIN / 4);
    int16_t error_int = target_int - actual_int;

    txLength = 0;
    txBusy = 1;
    txPos = 0;

    send_int_to_tx(target_int);
    send_char_to_tx(' ');

    send_int_to_tx(actual_int);
    send_char_to_tx(' ');

    send_int_to_tx(error_int);

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
    configureTIM3();
    configureTIM4();
    configureTIM5();
    configureTIM6();
    configureDMA1();
    configureUSART2();
    configureNVIC();

    while (1) {
        if (telemetryFlag) {
            control_step();
            if (!txBusy) {
                send_telemetry();
            }
            telemetryFlag = false;
        } if (readMotorSpeedFLag) {
            current_motor_speed = get_motor_speed();
            readMotorSpeedFLag = false;
        }
    }
}

void TIM5_IRQHandler(void) {
    if (TIM5->SR & TIM_SR_UIF) {
        telemetryFlag = true;
        TIM5->SR &= ~TIM_SR_UIF;
    }
}

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        readMotorSpeedFLag = true;
        TIM6->SR &= ~TIM_SR_UIF;
    }
}

void DMA1_Stream6_IRQHandler(void) {
    if ((DMA1->HISR & (0x1 << 21))) {  // флаг TC (uart tx)
        txBusy = false;
        DMA1->HIFCR |= (0x1 << 21);
    }
}
