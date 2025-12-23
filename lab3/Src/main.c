#include <stdint.h>
#include <stm32f446xx.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#define HSE_SPEED               8000000UL
#define BAUDRATE                1000000
#define V_REF                   3.3f

// example of packet: 0xAA 0x01 0x0 0x08 0x1 0x55

#define PACKET_SIZE             6 // header, slave_id, data[1] (A), data[2] (omega), terminator
#define PACKET_HEADER           0xAA
#define PACKET_TERM             0x55
#define TIM_FREQ                1000.0f  // frequency of update interrupt of TIM in hz
#define VOLTAGE_START           1.65f

#define PACKET_TERM_POS         5
#define DATA_2_POS              4
#define DATA_1_POS              2
#define PACKET_SLAVE_ID_POS     1
#define PACKET_HEADER_POS       0

#define WAVE_PERIOD 66  // за один период колебаний напряжение меняется 1000 раз (чтобы соответствовало реальному требованию синуса частоты 15гц из задания, надо чтобы это значение совпадало с обратной величиной к частоте возникновения update interrupt, то есть если CLK_PSC=1мгц, а ARR = 1000, то частота возникновения update = 1кгц, и WAVE_PERIOD = 1000)
#define WAVE_SIN 0x61
#define WAVE_SQUARE 0x62
#define WAVE_TRIANGLE 0x63
#define WAVE_SAW 0x64


int waveType = WAVE_SIN;
// float n1;
bool timerUpdate;
bool rxFlag;

uint8_t txBuffer[PACKET_SIZE];
unsigned short txIndex = 0;
bool txBusy = false; // если true - пакет (не отдельный байт!) ещё отправляется, писать другие пакеты нельзя.

uint8_t rxBuffer[PACKET_SIZE]; // в буфере храним все полученные байты пакета
unsigned short rxIndex = 0;
bool rxFlag = false;

unsigned short pktLength;

uint16_t n = 0;
uint16_t A_int = 2048U;  // амплитуда от 0 до 4095

float A = 1.5f;  // амплитуда в В
uint8_t nu = 15;  // частота в Гц

float omega;
float dt;
float half;
float period;
float voltage;

float_t sin_dots[WAVE_PERIOD];

void set_sin_dots(void) {
    for (int i = 0; i < WAVE_PERIOD; i++) {
        sin_dots[i] = (A / 2.0f) * sinf(omega * (float)i/1000.0f);
    }
}

void set_signal_parameters(void) {
    omega = nu * 2 * M_PI;
    dt = 1.0f/(float)TIM_FREQ;
    period = 1.0f/nu;
    half = (period/2.0f);
}

void configureRCC(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));  // wait until HSE is ready

    // настройка предделителей шин (по дефолту 1 везде)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 clock is set to 50/2 т.к. max(APB1_CLK) = 45 MHz

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLSRC_Msk);
    RCC->PLLCFGR |= (RCC_PLLCFGR_PLLM_3);  // PLLM = 8
    RCC->PLLCFGR |= (100 << RCC_PLLCFGR_PLLN_Pos);  // PLLN = 100
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_0;  // PLLP = 4
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
}

void configureFLASH(void) {
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_1WS;  // либо эквивалентное значение 1
}

void enableFPU(void) {
    SCB->CPACR |= (3UL << 10 * 2) | (3UL << 11 * 2);  // разобрать как работает FPU
}

void configureUSART(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; // переводим пины PA2 и PA3 в alternate mode
    // Установить PA3 в режим Pull-up
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD3_Msk); 
    GPIOA->PUPDR |= (1 << GPIO_PUPDR_PUPD3_Pos); // '01'b = Pull-up
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = 25000000/BAUDRATE;  // задаём baud rate, freq of apb1 = HCLK / 2 = 25MHz
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;  // включаем transmitter, receiver
    // USART2->CR1 |= (1 << 15);  // oversampling by 8
    USART2->CR1 |= USART_CR1_RXNEIE;  // разрешаем прерывания для приёмника (на уровне периферии)
    USART2->CR1 |= USART_CR1_UE;  // разрешаем сам USART
    NVIC_EnableIRQ(USART2_IRQn);
    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);
}

void sendByte(uint8_t byte) {
    if (txBusy) return; 
    pktLength = 1;
    txBuffer[txIndex++] = byte;
    txIndex = 0;
    txBusy = true;
    USART2->CR1 |= USART_CR1_TXEIE;
}

void configureDAC(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(0x3 << 8);
    GPIOA->MODER |= (0x3 << 8);  // analog mode for PA4 (DAC1 out)
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    // коннект с таймером
    // DAC->CR |= DAC_CR_TEN1;  // trigger enable
    // TIM6 выбран потому что он внутренне соединен с DAC (как именно?)
    // DAC->CR &= ~(DAC_CR_TSEL1_Msk);  // 000: TRGO 6 event

    DAC1->CR |= DAC_CR_EN1;
    while ((DAC->CR & DAC_CR_EN1_Msk) != DAC_CR_EN1);
    DAC1->DHR12R1 = (uint16_t)VOLTAGE_START * 4095;
}

void configureTIM6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = (25*2 - 1);  // частота обновления counter register 1мгц (APB1 prescaler = 2 (!= 1); => частота на таймере в 2 раза больше чем на всей шине)
    TIM6->ARR = 0x3E8;  // переполнение счетчика каждую мс (за мс напряжение из DOR1 точно установится, t_settling около 5мкс)
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
    // TIM6->CR2 &= (TIM_CR2_MMS_Msk);
    // при каждом появлении update event должен запускаться ЦАП
    // TIM6->CR2 |= TIM_CR2_MMS_1;  // The update event is selected as a trigger output
}

void enableNVIC_IRQs(void) {
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

int main(void) {
    configureFLASH();
    configureRCC();
    enableFPU();
    set_signal_parameters();
    set_sin_dots();
    configureUSART();
    configureDAC();
    configureTIM6();
    enableNVIC_IRQs();
    while(1) {
        if (timerUpdate) {
            float n1 = (float)n * dt; 
            switch (waveType) {
                case WAVE_SIN:
                    voltage = VOLTAGE_START + sin_dots[n];
                    break;
                case WAVE_SQUARE:
                    voltage = (n1 < half) ? (VOLTAGE_START + A/2) : (VOLTAGE_START - A/2);
                    break;
                case WAVE_TRIANGLE: {
                    if (n1 < half) {
                        voltage = VOLTAGE_START - (A/2) + (2.0f * (A/2) * n1) / half;
                    } else {
                        voltage = VOLTAGE_START + (A/2) - (2.0f * (A/2) * (n1 - half)) / half;
                    }
                    break;
                }
                case WAVE_SAW:
                    voltage = VOLTAGE_START / 2 + A * ((float)n / WAVE_PERIOD);
                    break;
            }
            DAC->DHR12R1 = (uint16_t)(voltage / V_REF * 4095.0f);  // output data becomes available after the settling time is passed (t_settl depends on the voltage and the load, < 10мкс) 
            timerUpdate = false;
        }
        if (rxFlag) {
            if ((rxBuffer[PACKET_TERM_POS] == PACKET_TERM) & (rxBuffer[PACKET_HEADER_POS] == PACKET_HEADER)) {
                // отправлять значения двумя байтами, A переводится в диапазон 0-3.3, omega просто принимает значение до 256 (отправил 0x08  - частота 8 гц)
                uint8_t A_LB = rxBuffer[DATA_1_POS];
                uint8_t A_HB = rxBuffer[DATA_1_POS + 1];
                A_int = (uint16_t)(A_HB << 8) | A_LB;
                A = ((float)A_int/4095) * V_REF;

                nu = rxBuffer[DATA_2_POS];

                set_signal_parameters();
                set_sin_dots();
                sendByte(A_LB); 
                sendByte(A_HB);
                sendByte(nu);
                rxIndex = 0;
            }
            rxFlag = false;
        }
    }
}

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        timerUpdate = true;
        if (n++ == (uint32_t)WAVE_PERIOD-1) {n = 0;}
        TIM6->SR &= ~TIM_SR_UIF_Msk;
        // if (n++ == (uint32_t)WAVE_PERIOD) {n = 0;}  // возможно на 1 меньше
    }
}

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        uint8_t byte = USART2->DR;
        if (rxIndex >= PACKET_SIZE) {
            rxIndex = 0;
        }
        rxBuffer[rxIndex++] = byte;
        if (rxIndex == PACKET_SIZE) {
            rxFlag = true; // пакет полностью принят
        }
    } else if ((USART2->SR & USART_SR_TXE) && txBusy) { // заходим сюда только когда линия пуста и цепочка байтов уже есть в буфере линии TX
        if (txIndex < pktLength) {
            USART2->DR = txBuffer[txIndex++];
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
            txBusy = false; // передача завершена
            txIndex = 0;
        }
    }
}

void HardFault_Handler(void) {
    while (1) {}
}
