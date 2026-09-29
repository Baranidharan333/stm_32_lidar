#pragma once

// 1 Mbaud 8N1 UART on arbitrary GPIOG pins, for when the LiDAR's TX/RX land on pins no hardware
// UART can use (here: LiDAR TX on PG14 = D1, LiDAR RX on PG9 = D0, the reverse of USART6).
//
//   RX: TIM8 update events trigger DMA2 Stream1 (channel 7) to copy the upper byte of GPIOG->IDR into
//       a circular buffer at 4 samples per bit. available()/read() decode frames from that buffer, so
//       nothing is lost while the CPU is busy, as long as it polls at least every ~8 ms.
//   TX: bytes are queued; each is expanded to 10 GPIOG->BSRR words (start, 8 data, stop) which DMA2 Stream5
//       copies to the pin on TIM1 update events at the baud rate. write() never waits.

#include <Arduino.h>

class DmaSoftUart : public HardwareSerial {
public:
    // rxBit/txBit: pin numbers inside GPIOG (PG14 -> 14, PG9 -> 9)
    DmaSoftUart(uint8_t rxBit, uint8_t txBit) : rxBit_(rxBit), txBit_(txBit) {}

    void begin(unsigned long baud) override;
    void begin(unsigned long baud, uint16_t) override { begin(baud); }
    void end() override;
    int available() override;
    int peek() override;
    int read() override;
    void flush() override {}                          // never waits; TX drains in the background
    size_t write(uint8_t b) override;
    size_t write(const uint8_t *buf, size_t n) override;
    using Print::write;
    operator bool() override { return true; }

    uint32_t framingErrors() const { return framingErrors_; }
    uint32_t overruns() const { return overruns_; }
    uint32_t bytesReceived() const { return bytes_; }

private:
    static const uint32_t SAMPLES_PER_BIT = 4;
    static const uint32_t DMA_SIZE = 32768;             // 8 ms of samples at 4 MHz
    static const uint32_t RX_SIZE  = 4096;

    void decode();
    void kickTx();                                      // start DMA on queued bytes if idle

    uint8_t  rxBit_, txBit_;
    uint8_t  rxMask_ = 0;                               // bit of the sampled byte (upper byte of IDR)
    uint32_t baud_ = 1000000;

    uint8_t  dma_[DMA_SIZE] __attribute__((aligned(4)));
    uint32_t lastPos_ = 0;                              // DMA write position at the last decode
    uint32_t lastDecodeUs_ = 0;
    uint32_t headAbs_ = 0;                              // samples written by DMA, running count
    uint32_t tailAbs_ = 0;                              // samples consumed by the decoder

    static const uint32_t TX_QUEUE = 64;               // bytes
    static const uint32_t TX_CHUNK = 24;               // bytes per DMA transfer
    uint8_t  txq_[TX_QUEUE];
    uint16_t txHead_ = 0, txTail_ = 0;
    uint32_t txWords_[TX_CHUNK * 10 + 1];

    uint8_t  rx_[RX_SIZE];
    uint16_t rxHead_ = 0, rxTail_ = 0;

    uint32_t framingErrors_ = 0;
    uint32_t overruns_ = 0;
    uint32_t bytes_ = 0;
};
