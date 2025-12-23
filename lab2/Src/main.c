#include <stdint.h>
#include <stdbool.h>
#include <stm32f446xx.h>


//define APBx frequency
#define APB1_FREQ               16000000
//define UART Baudrate
#define BAUDRATE                9600
//define message parameters
#define MAX_PACKET_SIZE         16
#define PACKET_HEADER           0xAA
#define PACKET_TERM             0x55
#define MAX_DATA_SIZE           6 // максимальный допустимый размер массива передаваемых данных
#define SERVICE_BYTE_NUMBER     4 // кол-во служебных байт (не data)


#define DATA_START_POS          3
#define PACKET_SLAVE_ID_POS     2
#define PACKET_SIZE_POS         1
#define PACKET_HEADER_POS       0


// возможно эти команды надо как-то по-умному обозначать а не просто битами, чтобы было понятно что возвращается
#define ECHO                    0x61
#define ENABLE_ALL              0x62
#define DISABLE_ALL             0x63
#define EXACT_PATTERN           0x64
#define GET_STATE               0x65


typedef struct {
    uint8_t header;
    uint8_t size; // размер пакета в байтах
    uint8_t slave_id;
    uint8_t data[MAX_DATA_SIZE];
    uint8_t terminator;
} Packet_t;


bool transfer_complete = false;
uint32_t data_to_transfer;  // число ячеек данных для отправки по DMA (rx), обновлять при каждом присвоении NDTR для линии rx


uint8_t txBuffer[MAX_PACKET_SIZE];
unsigned short txIndex = 0;
bool txBusy = false; // если true - пакет (не отдельный байт!) ещё отправляется, писать другие пакеты нельзя.


uint8_t rxBuffer[MAX_PACKET_SIZE]; // в буфере храним все полученные байты пакета
unsigned short rxIndex = 0;
bool rxFlag = false;


// usart последовательный - только один пакет передаётся одновременно, можем использовать глоб. пер.
unsigned short pktLength;


void sendPacket(Packet_t *pkt) {
    if (txBusy) return;
    pktLength = pkt->size;
    txBuffer[txIndex++] = pkt->header;
    txBuffer[txIndex++] = pkt->size;
    txBuffer[txIndex++] = pkt->slave_id;
    for (int i = 0; i < (pkt->size - SERVICE_BYTE_NUMBER); i++) {
        txBuffer[txIndex++] = pkt->data[i];
    }
    txBuffer[txIndex++] = pkt->terminator;
    DMA1_Stream6->NDTR = pktLength;  // amount of bytes to transfer
    DMA1_Stream6->M0AR = (uint32_t)txBuffer;  // address to buffer from which the data will be taken
    DMA1_Stream6->CR |= 0x1;  // stream 6 enable
    txIndex = 0;
    txBusy = true;
}


void sendByte(uint8_t byte) {
    if (txBusy) return;
    txBuffer[txIndex++] = byte;
    pktLength = 1;
    DMA1_Stream6->NDTR = pktLength;  // amount of bytes to transfer
    DMA1_Stream6->M0AR = (uint32_t)txBuffer;  // address to buffer from which the data will be taken
    DMA1_Stream6->CR |= 0x1;  // stream 6 enable
    txIndex = 0;
    txBusy = true;
}


int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;  // переводим пины PA2 и PA3 в alternate mode
    GPIOC->MODER |= 0x555500;  // переводим светодиоды в output
    // настройка триггеров для DMA
    DMA1_Stream5->CR &= ~DMA_SxCR_EN; // отключаем поток перед настройкой (нужно ли?)
    while (DMA1_Stream5->CR & DMA_SxCR_EN);
    DMA1_Stream5->CR &= ~(0x7 << 25);  // CHSEL clearing
    DMA1_Stream5->CR |= (0x1 << 27);  // channel 4 (USART2_RX) selected
    DMA1_Stream5->CR &= ~(0xC0);  // data transfer direction reset (peripheral-to-memory is default)
    DMA1_Stream5->NDTR = MAX_PACKET_SIZE;  // amount of bits to transfer (buffer size)
    DMA1_Stream5->PAR = (uint32_t)&USART2->DR;  // peripheral address where we read data from
    DMA1_Stream5->M0AR = (uint32_t)rxBuffer;  // address to buffer where the data will be written
    DMA1_Stream5->CR &= ~(0x3 << 11);  // PSIZE reset, MSIZE default is 8 bit (then MSIZE is equal to PSIZE)
    DMA1_Stream5->CR |= (0x1 << 8);  // circular mode
    DMA1_Stream5->CR |= (0x1 << 10);  // MINC: memory address pointer is incremented after each data transfer
    // DMA1_Stream5->CR |= (0x1 << 4);  // TCIE transfer complete IE


    DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6; // нужно ли?
    DMA1_Stream6->CR &= ~(0x7 << 25);  // chsel reset
    DMA1_Stream6->CR |= (0x1 << 27);  // channel 4 (USART2_TX) selected
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;  // peripheral address where we put data
    DMA1_Stream6->CR &= ~(0x3 << 6);  // data transfer direction reset
    DMA1_Stream6->CR |= (0x1 << 6);  // data transfer direction: memory-to-peripheral
    // DMA1_Stream6->CR |= (0x1 << 8);  // circular mode
    DMA1_Stream6->CR |= (0x1 << 10);  // MINC
    DMA1_Stream6->CR |= (0x1 << 4);  // TCIE transfer complete IE
    DMA1_Stream6->CR &= ~(0x3 << 11);  // PSIZE reset, MSIZE default is 8 bit (then MSIZE is equal to PSIZE)


    USART2->BRR |= APB1_FREQ/BAUDRATE;  // задаём baud rate
    USART2->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;  // DMA enable transmitter + reciever
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;  // включаем transmitter, receiver
    // USART2->CR1 |= USART_CR1_RXNEIE;  // разрешаем прерывания для приёмника (на уровне периферии)
    USART2->CR1 |= USART_CR1_IDLEIE;  // разрешаем прерывания для состояния простоя
    USART2->CR1 |= USART_CR1_UE;  // разрешаем сам USART
    DMA1_Stream5->CR |= 0x1;  // stream 5 enable
    NVIC_EnableIRQ(USART2_IRQn);  // разрешаем прерывания в NVIC
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) | (7 << GPIO_AFRL_AFSEL3_Pos);
   
    while(true) {
        if (rxFlag) {
            DMA1_Stream5->CR &= ~(0x1);  // отключение dma1s5
            uint32_t packet_size = rxBuffer[PACKET_SIZE_POS];
            if ((rxBuffer[PACKET_HEADER_POS] == PACKET_HEADER) & (rxBuffer[packet_size-1] == PACKET_TERM)) {
                switch (rxBuffer[DATA_START_POS]) {
                    case ECHO: {
                        static Packet_t echoPkt;  // без static значения внутри пакета будут мусорными (локальные переменные на стеке не инициализируются автоматически)
                        echoPkt.header = rxBuffer[PACKET_HEADER_POS];
                        echoPkt.size = rxBuffer[PACKET_SIZE_POS];
                        echoPkt.slave_id = rxBuffer[PACKET_SLAVE_ID_POS];
                        for (int i = 0; i < (echoPkt.size - SERVICE_BYTE_NUMBER); i++) {
                            echoPkt.data[i] = rxBuffer[DATA_START_POS + i];
                        }
                        echoPkt.terminator = rxBuffer[echoPkt.size - 1]; // терминатор — последний байт
                        sendPacket(&echoPkt);
                        break;
                    } case ENABLE_ALL: {
                        GPIOC->BSRR |= 0xFF0; // переводим пины 4-11 порта C в верхнее положение (светодиоды)
                        sendByte(ENABLE_ALL);
                        break;
                    } case DISABLE_ALL: {
                        GPIOC->BSRR |= 0xFF0 << 16; // пины 4-11 порта C отключаем, остальные не поменяются
                        sendByte(DISABLE_ALL);
                        break;
                    } case EXACT_PATTERN: {
                        if (rxIndex - SERVICE_BYTE_NUMBER == 2) { // действительно ли пришло 2 байта данных? (команда+параметр)
                            uint8_t ledNumber = rxBuffer[DATA_START_POS + 1];
                            GPIOC->BSRR |= 0xFF0 << 16;
                            GPIOC->BSRR |= 0x8 << ledNumber;  // 0x8 потому что светодиоды не по 0-7 пинам, а по 4-11
                            sendByte(EXACT_PATTERN);
                        }
                        break;
                    } case GET_STATE: {
                        // uint8_t lcdState = (GPIOC->ODR & (0xFF << 4)) >> 4; // FF - первые 8 бит, сдвиг на 4 чтоб получить не 0-7, а 4-11, потом вернуть в начальные позиции (используется младший байт)
                        uint8_t lcdState = 0x0;
                        sendByte(lcdState);
                        break;
                    }
                }
            }
            for (int i = 0; i < MAX_PACKET_SIZE; i++) {
                rxBuffer[i] = 0;
            }
            rxIndex = 0;
            rxFlag = false;
            DMA1_Stream5->M0AR = (uint32_t)rxBuffer;
            DMA1_Stream5->CR |= (0x1);
        }
    }
}


void DMA1_Stream6_IRQHandler(void) {
    if ((DMA1->HISR & (0x1 << 21))) {  // флаг TC на 6 потоке
        txBusy = false;
        DMA1->HIFCR |= (0x1 << 21);
    }
}


void USART2_IRQHandler(void) {
    if ((USART2->SR & USART_SR_IDLE)) {
        // простаиваем и передача пакета завершена, обрабатываем
        (void)USART2->SR;
        (void)USART2->DR;  // флаг USART_SR_IDLE сбрасывается чтением регистра данных
        rxFlag = true;
    }
}
