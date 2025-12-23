#include <stdint.h>
#include <stdbool.h>

// define директива идентификатор посл-сть символов которые подставляются, макрос на выходе
// define ПРОСТО подставляет символы

#define BITBAND_PERIPH_REF   0x40000000U
#define BITBAND_PERIPH_BASE  0x42000000U
#define PERIPH_BASE          0x40000000U // макрос, содержащий базовый адрес области хранения бит УВВ
// шины
#define AHB1PERIPH_BASE     (PERIPH_BASE + 0x20000U)
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x10000U)
// GPIO и RCC, и всё содержащее регистры - периферия. GPIOC - конкретный блок периферии
#define GPIOC_BASE          (AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASE          (AHB1PERIPH_BASE + 0x0C00U)
#define RCC_BASE            (AHB1PERIPH_BASE + 0x3800U)
#define SYSCFG_BASE         (APB2PERIPH_BASE + 0x3800U)
#define EXTI_BASE           (APB2PERIPH_BASE + 0x3C00U)
#define TIM2_BASE           (0x40000000U) // забавно что с базовыми адресами периферии и bit-banding совпадает

// функциональный макрос
#define BITBAND_PERIPH(addr, bit) \
    ((volatile uint32_t*)(BITBAND_PERIPH_BASE + ((addr - BITBAND_PERIPH_REF) * 32U) + (bit * 4U)))

#define c13 !(*BITBAND_PERIPH(0x40020810UL, 13))
#define d2 !(*BITBAND_PERIPH(0x40020C10UL, 2))

// just Ctrl+C -> Ctrl+V from CMSIS
#define NVIC_ISER_BASE  0xE000E100UL
void NVIC_EnableIRQ(uint32_t irq_num) {
    volatile uint32_t *reg = (uint32_t *)(NVIC_ISER_BASE + (irq_num / 32) * 4);
    *reg = (1U << (irq_num % 32));
}

typedef struct // регистры внутри блока GPIO (регистр = 32-битная ячейка памяти)
{
    uint32_t MODER;    // 0x00
    uint32_t OTYPER;   // 0x04
    uint32_t OSPEEDR;  // 0x08
    uint32_t PUPDR;    // 0x0C
    uint32_t IDR;      // 0x10
    uint32_t ODR;      // 0x14
    uint32_t BSRR;     // 0x18
    uint32_t LCKR;     // 0x1C
    uint32_t AFR[2];   // 0x20-0x24
} GPIO_TypeDef; 

typedef struct
{
    uint32_t CR;            // 0x00
    uint32_t PLLCFGR;       // 0x04
    uint32_t CFGR;          // 0x08
    uint32_t CIR;           // 0x0C
    uint32_t AHB1RSTR;      // 0x10
    uint32_t AHB2RSTR;      // 0x14
    uint32_t AHB3RSTR;      // 0x18
    uint32_t RESERVED0;     // Reserved, 0x1C
    uint32_t APB1RSTR;      // 0x20
    uint32_t APB2RSTR;      // 0x24
    uint32_t RESERVED1[2];  // Reserved, 0x28-0x2C
    uint32_t AHB1ENR;       // 0x30
    uint32_t AHB2ENR;       // 0x34
    uint32_t AHB3ENR;       // 0x38
    uint32_t RESERVED2;     // Reserved, 0x3C
    uint32_t APB1ENR;       // 0x40
    uint32_t APB2ENR;       // 0x44
    uint32_t RESERVED3[2];  // Reserved, 0x48-0x4C
    uint32_t AHB1LPENR;     //
    uint32_t AHB2LPENR;     // 0x54
    uint32_t AHB3LPENR;     // 0x58
    uint32_t RESERVED4;     // Reserved, 0x5C
    uint32_t APB1LPENR;     // 0x60
    uint32_t APB2LPENR;     // 0x64
    uint32_t RESERVED5[2];  // 0x68-0x6C
    uint32_t BDCR;          // 0x70
    uint32_t CSR;           // 0x74
    uint32_t RESERVED6[2];  // 0x78-0x7C
    uint32_t SSCGR;         // 0x80 
    uint32_t PLLI2SCFGR;    // 0x84
    uint32_t PLLSAICFGR;    // 0x88
    uint32_t DCKCFGR;       // 0x8C
    uint32_t CKGATENR;      // 0x90
    uint32_t DCKCFGR2;      // 0x94
} RCC_TypeDef;

typedef struct {
    uint32_t MEMRMP;      // 0x00
    uint32_t PMC;         // 0x04
    uint32_t EXTICR[4];   // 0x08-0x14
} SYSCFG_TypeDef;

typedef struct {
    uint32_t IMR;      // 0x00
    uint32_t EMR;      // 0x04
    uint32_t RTSR;     // 0x08
    uint32_t FTSR;     // 0x0C
    uint32_t SWIER;    // 0x10
    uint32_t PR;       // 0x14
} EXTI_TypeDef;

typedef struct {
    uint32_t CR1;
    uint32_t CR2;
    uint32_t SMCR;
    uint32_t DIER;
    uint32_t SR;
    uint32_t EGR;
    uint32_t CCMR1;
    uint32_t CCMR2;
    uint32_t CCER;
    uint32_t CNT;
    uint32_t PSC;
    uint32_t ARR;
} TIM_TypeDef;

#define GPIOC   ((GPIO_TypeDef*) GPIOC_BASE) // макрос, возвращающий указатель на структуру GPIO_TypeDef с базовым адресом GPIOC_BASE
#define RCC     ((RCC_TypeDef*) RCC_BASE)
#define GPIOD   ((GPIO_TypeDef*) GPIOD_BASE)
#define SYSCFG  ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI    ((EXTI_TypeDef *) EXTI_BASE)
#define TIM2    ((TIM_TypeDef*) TIM2_BASE)

#define EXTI15_10_IRQn  40
#define TIM2_IRQn       28

bool c13Flag = false;
bool d2Flag = false;

// для обработки дребезга
bool c13Omitted = false;
bool d2Omitted = false;
bool debounceActive = false;

void EXTI15_10_IRQHandler(void) {
    // обработчик прерывания по нажатиям на кнопки
    if (EXTI->PR & (0x1 << 13)) {
        EXTI->PR = (0x1 << 13);           // запись 1 в нужную линию регистра PR сбрасывает флаг прерывания
        c13Omitted = true;
        debounceActive = true;             // начался дребезг
    }
    if (EXTI->PR & (0x1 << 12)) {
        EXTI->PR = (0x1 << 12);
        d2Omitted = true;
        debounceActive = true;
    }

    if (debounceActive) {
        EXTI->IMR &= ~((0x1 << 12) | (0x1 << 13));  // блокируем EXTI
        TIM2->CNT = 0x0;      // Counter value, значение до которого считает счётчик
        TIM2->SR = 0x0;       // Update interrupt flag = 0
        TIM2->CR1 = 0x1;    // Counter enable
    }
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & 0x1) {
        TIM2->SR = 0x0;  // reset interruption flag
        TIM2->CR1 = 0x0;  // Disable counter

        // проверка состояния кнопок через 20 мс, bit-banding
        if (c13) {
            c13Flag = true;
        }
        else {
            c13Flag = false;
        }

        if (d2) {
            d2Flag = true;
        }
        else {
            d2Flag = false;
        }

        debounceActive = false;
        EXTI->IMR |= (0x1 << 12) | (0x1 << 13); // снова разрешаем прерывания
    }
}


int main(void)
{
    RCC->AHB1ENR |= 0xC; // 0xC = 0b1100, тактирование на 2 и 3 бит (порты C и D)
    RCC->APB2ENR |= (0x1 << 14); // SYSCFG (для EXTI), он под 14 номером в регистре

    GPIOC->MODER |= 0x555500; // 4-11 пины порта GPIOC (диоды) в output
    GPIOC->MODER &= ~(0x11 << (13 * 2));  // PC13, 00: Input
    GPIOD->MODER &= ~(0x11 << (2 * 2));  // PD2, 00: Input

    GPIOC->PUPDR &= ~(0x3 << (13 * 2));         // Сначала очищаем биты 26 и 27
    GPIOC->PUPDR |= (0x01 << (13 * 2));         // 01 - Pull-up
    GPIOD->PUPDR &= ~(0x3 << (2 * 2));
    GPIOD->PUPDR |= (0x01 << (2 * 2));

    // Мультиплексор
    // EXTICR - массив регистров настройки внешних прерываний, EXTICR[3] - регистр с 4 настройками
    // SYSCFG: EXTI12 <- PD2, EXTI13 <- PC13
    SYSCFG->EXTICR[3] &= 0x0; // Сначала очищаем
    SYSCFG->EXTICR[3] |= (0x2); // GPIOC (0x0010) в EXTI12 (в прерывание по 12 линии)
    SYSCFG->EXTICR[3] |= (0x3 << 4); // GPIOD (0x0011) в EXTI13
    
    // Разрешаем линии 12 и 13 EXTI
    EXTI->IMR  |= (0x1 << 12) | (0x1 << 13);   // Unmask interrupt, разрешает контроллеру EXTI реагировать на изменения сигналов с этих пинов
    EXTI->FTSR |= (0x1 << 12) | (0x1 << 13);   // ниспадающий фронт 
    EXTI->RTSR &= ~((0x1 << 12) | (0x1 << 13));// одновременно падающий и возрастающий фронт на кнопке? надо убрать по идее, не будет такой ситуации

    TIM2->PSC = 16000 - 1;  // при 16 МГц  1 мс (default for APB1 after reset (HCLK with PSC=1))
    TIM2->ARR = 20;         // 20 счетов = 20 мс
    TIM2->DIER = 0x1;       // UIE, разрешить прерывания update

    NVIC_EnableIRQ(EXTI15_10_IRQn);  // линии 10–15 обслуживаются этим IRQ
    NVIC_EnableIRQ(TIM2_IRQn);

    GPIOC->BSRR = 0x10; // зажигаем светодиод на PC4
    uint32_t mask = 0xFF << 4; // Биты PC4–PC11
    while(1) {
        uint32_t led_bits = GPIOC->ODR & mask;
        if (c13Flag) {
            if (led_bits & (1 << 11)) {  // светодиод PC11 активен
                GPIOC->BSRR = led_bits << 16;  // выключение текущего диода, и не чего-то кроме диодов
                GPIOC->BSRR |= (0x1 << 4);
            } else {
                GPIOC->BSRR = led_bits << 16; // тут именно led_bits надо сдвигать на 16, а не что-то ещё - в BSRR 16-31 биты за сброс позиции регистра данных GPIO отвечают, недостаточно просто обнулить 0-15 биты
                GPIOC->BSRR |= led_bits << 1;
            }
        }
        else if (d2Flag) {
            if (led_bits & (1 << 4)) {  // светодиод PC4 активен
                GPIOC->BSRR = led_bits << 16;
                GPIOC->BSRR |= (0x1 << 11);
            } else {
                GPIOC->BSRR = led_bits << 16;
                GPIOC->BSRR |= led_bits >> 1;
            }
        }
    }
}
