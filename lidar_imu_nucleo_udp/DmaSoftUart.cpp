#include "DmaSoftUart.h"

void DmaSoftUart::begin(unsigned long baud)
{
    // TX idles high
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    GPIOG->BSRR = 1u << txBit_;
    GPIOG->MODER = (GPIOG->MODER & ~(3u << (2 * txBit_))) | (1u << (2 * txBit_));        // output
    GPIOG->OSPEEDR |= 3u << (2 * txBit_);
    // RX input with pull-up
    GPIOG->MODER &= ~(3u << (2 * rxBit_));
    GPIOG->PUPDR = (GPIOG->PUPDR & ~(3u << (2 * rxBit_))) | (1u << (2 * rxBit_));

    baud_ = baud;

    // Sample the byte of IDR that holds the RX pin
    volatile uint8_t *idrByte = (volatile uint8_t *)&GPIOG->IDR + (rxBit_ / 8);
    rxMask_ = 1u << (rxBit_ % 8);

    // TIM8 update at baud * 4. TIM8 runs from APB2, at 2x PCLK2 when APB2 is divided.
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    uint32_t timClk = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0) timClk *= 2;
    TIM8->CR1 = 0;
    TIM8->PSC = 0;
    TIM8->ARR = timClk / (baud * SAMPLES_PER_BIT) - 1;
    TIM8->EGR = TIM_EGR_UG;
    TIM8->DIER = TIM_DIER_UDE;

    // DMA2 Stream1 channel 7 = TIM8_UP: peripheral (GPIO byte) -> circular memory buffer
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    DMA2_Stream1->CR = 0;
    while (DMA2_Stream1->CR & DMA_SxCR_EN) {}
    DMA2->LIFCR = 0x3Du << 6;                           // clear stream 1 flags
    DMA2_Stream1->PAR  = (uint32_t)idrByte;
    DMA2_Stream1->M0AR = (uint32_t)dma_;
    DMA2_Stream1->NDTR = DMA_SIZE;
    DMA2_Stream1->FCR  = 0;                             // direct mode, byte to byte
    DMA2_Stream1->CR   = (7u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_CIRC;
    DMA2_Stream1->CR  |= DMA_SxCR_EN;

    // TX: TIM1 update at the baud rate -> DMA2 Stream5 channel 6 copies BSRR words to the pin
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->CR1 = 0;
    TIM1->PSC = 0;
    TIM1->ARR = timClk / baud - 1;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->DIER = TIM_DIER_UDE;
    DMA2_Stream5->CR = 0;
    while (DMA2_Stream5->CR & DMA_SxCR_EN) {}
    DMA2->HIFCR = 0x3Du << 6;                           // clear stream 5 flags
    DMA2_Stream5->PAR = (uint32_t)&GPIOG->BSRR;
    DMA2_Stream5->FCR = 0;
    TIM1->CR1 = TIM_CR1_CEN;
    txHead_ = txTail_ = 0;

    lastPos_ = headAbs_ = tailAbs_ = 0;
    lastDecodeUs_ = micros();
    rxHead_ = rxTail_ = 0;
    TIM8->CR1 = TIM_CR1_CEN;
}

void DmaSoftUart::end()
{
    TIM8->CR1 = 0;
    DMA2_Stream1->CR = 0;
    TIM1->CR1 = 0;
    DMA2_Stream5->CR = 0;
}

size_t DmaSoftUart::write(uint8_t b)
{
    return write(&b, 1);
}

// Queue bytes (dropped if the queue is full; commands are a few bytes) and start sending if idle
size_t DmaSoftUart::write(const uint8_t *buf, size_t n)
{
    size_t queued = 0;
    for (; queued < n; queued++) {
        uint16_t next = (txHead_ + 1) % TX_QUEUE;
        if (next == txTail_) break;
        txq_[txHead_] = buf[queued];
        txHead_ = next;
    }
    kickTx();
    return queued;
}

void DmaSoftUart::kickTx()
{
    if (DMA2_Stream5->CR & DMA_SxCR_EN) return;         // previous chunk still going out
    if (txHead_ == txTail_) return;

    uint32_t set = 1u << txBit_, reset = 1u << (txBit_ + 16);
    // Lead with one idle (high) bit: a timer request may already be pending when the stream is
    // enabled, which would cut the first slot short; that must not be the start bit.
    uint32_t words = 0;
    txWords_[words++] = set;
    while (txTail_ != txHead_ && words + 10 <= TX_CHUNK * 10 + 1) {
        uint32_t frame = (1u << 9) | ((uint32_t)txq_[txTail_] << 1);   // start 0, data LSB first, stop 1
        for (int i = 0; i < 10; i++) txWords_[words++] = (frame & (1u << i)) ? set : reset;
        txTail_ = (txTail_ + 1) % TX_QUEUE;
    }
    DMA2->HIFCR = 0x3Du << 6;
    DMA2_Stream5->M0AR = (uint32_t)txWords_;
    DMA2_Stream5->NDTR = words;
    DMA2_Stream5->CR = (6u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
                       DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_PL_1;
    DMA2_Stream5->CR |= DMA_SxCR_EN;
}

// Turn new DMA samples into bytes. A frame starts at the first low sample; bit k is read
// SAMPLES_PER_BIT * k + 2 samples later, i.e. near the middle of each bit.
void DmaSoftUart::decode()
{
    uint32_t pos = DMA_SIZE - DMA2_Stream1->NDTR;
    if (pos >= DMA_SIZE) pos = 0;
    uint32_t nowUs = micros();
    uint32_t bufferUs = DMA_SIZE * 1000000ULL / baud_ / SAMPLES_PER_BIT;
    if (nowUs - lastDecodeUs_ > bufferUs - 1000) {
        // Not polled for about a whole buffer (e.g. a long delay()): the DMA has wrapped, so the position
        // no longer tells how much is new and the buffer holds stale samples. Drop it and start fresh.
        if (headAbs_ != tailAbs_) overruns_++;
        lastPos_ = pos;
        tailAbs_ = headAbs_;
    }
    lastDecodeUs_ = nowUs;
    headAbs_ += (pos - lastPos_ + DMA_SIZE) % DMA_SIZE;
    lastPos_ = pos;

    if (headAbs_ - tailAbs_ > DMA_SIZE - 256) {         // decoder fell behind, samples overwritten
        tailAbs_ = headAbs_ - 64;
        overruns_++;
    }

    const uint32_t M = DMA_SIZE - 1;
    const uint32_t FRAME = SAMPLES_PER_BIT * 9 + 2;     // up to the middle of the stop bit
    while (true) {
        // Skip the idle (high) line, 4 samples per step where aligned
        const uint32_t idle4 = rxMask_ * 0x01010101u;
        while (tailAbs_ != headAbs_ && (tailAbs_ & 3)) {
            if (!(dma_[tailAbs_ & M] & rxMask_)) break;
            tailAbs_++;
        }
        while (headAbs_ - tailAbs_ >= 4 && !(tailAbs_ & 3) &&
               (*(const uint32_t *)&dma_[tailAbs_ & M] & idle4) == idle4) tailAbs_ += 4;
        while (tailAbs_ != headAbs_ && (dma_[tailAbs_ & M] & rxMask_)) tailAbs_++;
        if (headAbs_ - tailAbs_ <= FRAME) return;       // wait for the whole frame

        uint32_t s = tailAbs_;
        if (dma_[(s + 1) & M] & rxMask_) {              // glitch, not a start bit
            tailAbs_ = s + 1;
            continue;
        }
        uint8_t b = 0;
        for (uint32_t k = 0; k < 8; k++)
            if (dma_[(s + SAMPLES_PER_BIT * (k + 1) + 2) & M] & rxMask_) b |= 1u << k;

        if (dma_[(s + FRAME) & M] & rxMask_) {
            uint16_t next = (rxHead_ + 1) % RX_SIZE;
            if (next != rxTail_) {
                rx_[rxHead_] = b;
                bytes_++;
                rxHead_ = next;
            }
        } else {
            framingErrors_++;
        }
        tailAbs_ = s + FRAME;
    }
}

int DmaSoftUart::available()
{
    kickTx();
    decode();
    return (rxHead_ - rxTail_ + RX_SIZE) % RX_SIZE;
}

// peek()/read() take already decoded bytes; they only decode when none are left (available() decodes)
int DmaSoftUart::peek()
{
    if (rxHead_ == rxTail_ && !available()) return -1;
    return rx_[rxTail_];
}

int DmaSoftUart::read()
{
    if (rxHead_ == rxTail_ && !available()) return -1;
    uint8_t b = rx_[rxTail_];
    rxTail_ = (rxTail_ + 1) % RX_SIZE;
    return b;
}
