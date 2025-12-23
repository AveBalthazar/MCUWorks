// лаб 7
// снести настройку PB1 (EXT_IO1), PB2 (EXT_IO2), PB3 (CS_MEM), PB12 (CS_EXT), проверить запустится ли оно
// удалить комментарий из configureSPI

#include <stdint.h>
#include <stm32f446xx.h>
#include "stdbool.h"

/*
по юарту можно отправить пакет вида 0xAA DIGIT DIGIT_POS, где DIGIT - желаемая цифра, DIGIT_POS - разряд (позиция) на который хотим её вывести.
после этого посредством SPI данные уйдут в два сдвиговых регистра, первый байт отправится в регистр, задающий сегмент,
а второй - в регистр, задающий разряд (позицию), и должен зажечься нужный разряд в индикаторе (цифра, которая была отправлена).
*/

#define HSE_SPEED                   8U                                  // HSE frequency in MHz
#define HCLK                        50                                  // desired HCLK frequency in MHz
#define PLLM                        (HSE_SPEED/2U)
#define PLLN                        100U
#define PLLP                        ((HSE_SPEED / PLLM) * PLLN) / HCLK
#define PPRE1                       RCC_CFGR_PPRE1_DIV2                 // prescaler for APB1
#define PPRE1_INT                   2U                                  // prescaler for APB1 (used in funcs)
#define PPRE2                       RCC_CFGR_PPRE2_DIV2                 // prescaler for APB2
#define PPRE2_INT                   2U                                  // prescaler for APB2 (used in funcs)
#define APB1_FREQ                   HCLK * 1000000U / PPRE1_INT         // APB1 frequency in hz
#define APB2_FREQ                   HCLK * 1000000U / PPRE2_INT         // APB2 frequency in hz
#define FLASH_LATENCY               1                                   // amount of wait states (from 1 to 15)
#define USART_BAUDRATE              115200U

#define TIM_PSC(PERIPH_FREQ, DESIRED_TIM_PSC_FREQ) ((PERIPH_FREQ/DESIRED_TIM_PSC_FREQ) - 1);
#define TIM_ARR(PERIPH_FREQ, TIM_PSC, DESIRED_TIM_FREQ) ((PERIPH_FREQ/(TIM_PSC + 1))/DESIRED_TIM_FREQ);
#define TIM3_FREQ                   1000U                                // desired frequency of refreshing values on indicator (in hz) 

#define PACKET_HEADER               0xAA
#define PACKET_SIZE                 3U
#define POSITION_POS                2U                                  // number of byte sent to usart which contains a desired digit position 
#define DIGIT_POS                   1U                                  // number of byte sent to usart which contains a desired digit
#define PACKET_HEADER_POS           0U                                  // number of byte sent to usart which contains a packet header

#define DIGITS_QUANTITY             8U                                  // amount of digits on indicator

#define POSITION_MASK_0             (~1)
#define POSITION_MASK_1             (~2)
#define POSITION_MASK_2             (~4)
#define POSITION_MASK_3             (~8)
#define POSITION_MASK_4             (~16)
#define POSITION_MASK_5             (~32)
#define POSITION_MASK_6             (~64)
#define POSITION_MASK_7             (~128)

static volatile uint8_t data_to_transfer[PACKET_SIZE];                  // contains a masked digit as a first byte and a digit position (0 to 7) as a second
static volatile bool newData = false;
static volatile bool timUpdate = false;
static const uint8_t segment_map[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
static uint8_t digits_list[DIGITS_QUANTITY];                            // contains digits which are seen on indicator. position in list is a digit position
static volatile uint8_t position_mask = 0;
uint8_t n = 0;

static void configureNVIC(void) {
    NVIC_EnableIRQ(SPI2_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
}

static void setFLASH(void) {
    FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;
    FLASH->ACR |= (FLASH_LATENCY << FLASH_ACR_LATENCY_Pos);
}

static void configureRCC(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    RCC->CFGR &= ~(RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE1_0);
    RCC->CFGR |= PPRE1;
    RCC->CFGR |= PPRE2;

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLSRC_Msk);
    RCC->PLLCFGR |= (PLLM << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (PLLN << RCC_PLLCFGR_PLLN_Pos); // VCO_clock
    RCC->PLLCFGR |= (((PLLP / 2) - 1) << RCC_PLLCFGR_PLLP_Pos);  // PLL_output
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);
}

static void configureGPIO(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN;

    GPIOB->MODER &= ~(GPIO_MODER_MODER13_Msk | GPIO_MODER_MODER15_Msk |
                      GPIO_MODER_MODER12_Msk | GPIO_MODER_MODER3_Msk  | GPIO_MODER_MODER2_Msk  | 
                      GPIO_MODER_MODER1_Msk);

    // PB13 (CLK), PB15 (MOSI) -> Alternate Function
    // PB1 (EXT_IO1), PB2 (EXT_IO2), PB3 (CS_MEM), PB12 (CS_EXT) -> Output
    GPIOB->MODER |= (GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1); // AF
    GPIOB->MODER |= (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER3_0  | GPIO_MODER_MODER2_0  | 
                      GPIO_MODER_MODER1_0); // Output

    // PA9, PA10, PA11 -> Output
    GPIOA->MODER &= ~(GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk | GPIO_MODER_MODER11_Msk);
    GPIOA->MODER |= (GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0);

    GPIOB->AFR[1] &= ~((0xFU << GPIO_AFRH_AFSEL13_Pos) | 
                       (0xFU << GPIO_AFRH_AFSEL15_Pos));
    GPIOB->AFR[1] |= (5U << GPIO_AFRH_AFSEL15_Pos |  // CLK
                      5U << GPIO_AFRH_AFSEL13_Pos);  // MOSI

    GPIOA->BSRR = GPIO_BSRR_BR_11; // ^OE = 0 (enable outputs)
    GPIOA->BSRR = GPIO_BSRR_BS_9;  // ^MR = 1 (unreset)

    // usart rx
    GPIOA->MODER |= GPIO_MODER_MODER3_1;
    GPIOA->AFR[0] |= 7 << GPIO_AFRL_AFSEL3_Pos;
    // может ещё настроить скорость (OSPEEDR)?
}

static void configureSPI(void) {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    SPI2->CR1 &= ~SPI_CR1_DFF_Msk;  // 8-bit
    SPI2->CR1 &= ~SPI_CR1_BR_Msk;  // SPI Prescaler = 2
    SPI2->CR1 |= SPI_CR1_MSTR;
    // если бы следующей строки не было, пришлось бы физически подключить SS к пину, и SPI сам отправлял бы "защелку" после отправки 8 бит, а нам надо отправить 2 раза по 8 бит
    SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;  // manual SS control 
    // SPI2->CR2 |= SPI_CR2_TXEIE;  // тут можно было TXDMAEN: Tx buffer DMA enable
    SPI2->CR1 |= SPI_CR1_SPE;
}

// по UART получаем числа с их позициями, которые необходимо выставить на индикаторе 
static void configureUSART(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = (APB1_FREQ / USART_BAUDRATE);
    USART2->CR3 |= USART_CR3_DMAR;
    USART2->CR1 |= USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
}

// DMA для обработки USART_RX
static void configureDMA1S5(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN);
    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
    DMA1_Stream5->CR &= ~DMA_SxCR_CHSEL_Msk;
    DMA1_Stream5->CR |= (4 << DMA_SxCR_CHSEL_Pos);
    DMA1_Stream5->CR &= ~DMA_SxCR_DIR_Msk;  // data transfer direction reset (peripheral-to-memory is default)
    DMA1_Stream5->NDTR = PACKET_SIZE;
    DMA1_Stream5->PAR = (uint32_t)&USART2->DR;
    DMA1_Stream5->M0AR = (uint32_t)data_to_transfer;
    DMA1_Stream5->CR &= ~DMA_SxCR_PSIZE_Msk;  // PSIZE reset, MSIZE default is 8 bit (then MSIZE is equal to PSIZE)
    DMA1_Stream5->CR |= DMA_SxCR_CIRC | DMA_SxCR_MINC | DMA_SxCR_EN;
}

// таймер для динамического обновления всех разрядов индикатора
void configureTIM3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;

    TIM3->CR1 &= ~TIM_CR1_CEN;
    uint32_t TIM3_PSC = TIM_PSC(APB1_FREQ*2, 1000000U);
    TIM3->PSC = TIM3_PSC;  // 1 мгц
    TIM3->ARR = TIM_ARR(APB1_FREQ*PPRE1_INT, TIM3_PSC, TIM3_FREQ);
    TIM3->SR &= ~TIM_SR_UIF;
    TIM3->DIER |= TIM_DIER_UIE;
}

// обновляет список выводимых цифр
static void update_digits_list(uint8_t digit, uint8_t position) {
    if ((digit <= 9) && (position <= 7)) {
        digits_list[position] = digit;
    }
}

static uint8_t get_position_mask(uint8_t position) {
    if (position <= 7) {
        switch (position) {
            case 0x0:
                return POSITION_MASK_0;
            case 0x1:
                return POSITION_MASK_1;
            case 0x2:
                return POSITION_MASK_2;
            case 0x3:
                return POSITION_MASK_3;
            case 0x4:
                return POSITION_MASK_4;
            case 0x5:
                return POSITION_MASK_5;
            case 0x6:
                return POSITION_MASK_6;
            case 0x7:
                return POSITION_MASK_7;
        }
    }
}

// отправляет указанную цифру по указанной позиции на индикатор
static void send_digit_to_indicator(uint8_t digit, uint8_t position) {
    if ((digit <= 9) && (position <= 7)) {
        GPIOA->BSRR = GPIO_BSRR_BR_10;
        uint8_t segments = segment_map[digit];
        
        // увы, на индикаторах бред при такой реализации, хоть и выглядит верно (видимо какие-то оптимизации компилятора связанные с тем что значение регистра 16 бит). но будет работать если вместо position отправлять константу. можно в дефайне хранить уже сдвинутое значение и отправлять его. но я не понял как без этого можно до ума довести :(
        // position_mask = ~(1 << position);  // отрицание т.к. транзисторы pnp открываются низким уровнем

        // рабочая альтернатива:
        position_mask = get_position_mask(position);

        while (!(SPI2->SR & SPI_SR_TXE));

        // байт идёт в дальний регистр U4
        SPI2->DR = segments;
        while (SPI2->SR & SPI_SR_BSY);
        while (!(SPI2->SR & SPI_SR_TXE));

        // байт идёт в ближний регистр U2
        SPI2->DR = position_mask;
        while (SPI2->SR & SPI_SR_BSY);

        // защелкиваем данные (SS/STCP single impulse)
        GPIOA->BSRR = GPIO_BSRR_BS_10;
        // добавить __NOP();? или среагирует?
    }
}

// задаёт начальное состояние для индикатора
static void digits_list_initialize(void) {
    for (uint8_t i = 0; i < DIGITS_QUANTITY; i++) {
        digits_list[i] = i;
    }
}

int main(void) {
    SCB->CPACR |= (3UL << (10*2)) | (3UL << (11*2));  // fpu enable
    setFLASH();
    configureGPIO();
    configureRCC();
    configureSPI();
    configureNVIC();
    configureUSART();
    configureDMA1S5();
    configureTIM3();

    digits_list_initialize();
    TIM3->CR1 |= TIM_CR1_CEN;

    while (1) { 
        if (newData) {
            update_digits_list(data_to_transfer[DIGIT_POS], data_to_transfer[POSITION_POS]);
            newData = false;
        }
        if (timUpdate) {
            if (n++ >= 7) n = 0;  // хз будет ли так работать корректно
            send_digit_to_indicator(digits_list[n], n);
            timUpdate = false;
        }
    }
}

void USART2_IRQHandler(void) {
    if ((USART2->SR & USART_SR_IDLE)) {
        // простаиваем и передача пакета завершена, обрабатываем
        (void)USART2->SR;
        (void)USART2->DR;  // флаг USART_SR_IDLE сбрасывается чтением регистра данных
        if (data_to_transfer[PACKET_HEADER_POS] == PACKET_HEADER){
            newData = true;
        }
    }
}

void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        timUpdate = true;
        TIM3->SR &= ~TIM_SR_UIF;
    }
}
