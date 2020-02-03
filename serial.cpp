//serial.c - Low level functions for sending and recieving bytes via the serial port

#include <avr/interrupt.h>//定义中断 ISR
#include "serial.h"
#include "config.h"
#include "motion_control.h"
#include "protocol.h"

//目测是环形队列格式，输出 和 接收 都是 采用环形队列，感觉会有一定难度
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer_head = 0;
uint8_t rx_buffer_tail = 0;

uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t tx_buffer_head = 0;

//tx_buffer_tail 前加了一个 volatile 声明，说明 tx_buffer_tail 可能会被 几个 子程序共享。
volatile uint8_t tx_buffer_tail = 0;

//用来判断 缓存的字节数量。
#ifdef ENABLE_XONXOFF
  volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable

  // Returns the number of bytes in the RX buffer. This replaces a typical byte counter to prevent
  // the interrupt and main programs from writing to the counter at the same time.
  static uint8_t get_rx_buffer_count()
  {
    if (rx_buffer_head == rx_buffer_tail) { return(0); }
    if (rx_buffer_head < rx_buffer_tail) { return(rx_buffer_tail-rx_buffer_head); }
    return (RX_BUFFER_SIZE - (rx_buffer_head-rx_buffer_tail));
  }
#endif

//串口初始化函数
void serial_init()
{
  // Set baud rate
  //这里需要查看 avr 串口设置资料，涉及到寄存器配置，我不是很熟悉，感觉和找到的代码有些区别，暂时先继续
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;

  // enable rx and tx
  //通过寄存器赋值实现，RXEN0 和 TXEN0 是寄存器
  UCSR0B |= 1<<RXEN0;
  UCSR0B |= 1<<TXEN0;

  // enable interrupt on complete reception of a byte
  //如果对 avr 串口编程很熟悉的话，上面的代码应该是 格式化的，就是遇到类似情况，copy paste 就可以了
  UCSR0B |= 1<<RXCIE0;

  // defaults to 8-bit, no parity, 1 stop bit
}

//将数据 data 写入串口
void serial_write(uint8_t data) {
  // Calculate next head
  //当next_head == TX_BUFFER_SIZE 时，表面走了一圈了，要再 从 0 开始
  uint8_t next_head = tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE) { next_head = 0; }

  // Wait until there is space in the buffer
  //当next_head == tx_buffer_tail 时，说明环形队列中已经没有空间了，在这里循环等待。循环内部判断 sys状态，避免死循环。
  while (next_head == tx_buffer_tail) {
    if (sys.execute & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  tx_buffer[tx_buffer_head] = data;
  tx_buffer_head = next_head;

  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR0B |=  (1 << UDRIE0);
}

// Data Register Empty Interrupt handler
//这是AVR的中断，USART0接收中断
#if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega2560__)
ISR(USART0_UDRE_vect)
#else
ISR(USART_UDRE_vect)
#endif
{
  // Temporary tx_buffer_tail (to optimize for volatile)
  // 数据从发送寄存器完整移动到移位寄存器
  // 中断或轮询模式，均是写数据清零
  // 每一个字符character发生一次
  uint8_t tail = tx_buffer_tail;

  #ifdef ENABLE_XONXOFF
    if (flow_ctrl == SEND_XOFF) {
      UDR0 = XOFF_CHAR;
      flow_ctrl = XOFF_SENT;
    } else if (flow_ctrl == SEND_XON) {
      UDR0 = XON_CHAR;
      flow_ctrl = XON_SENT;
    } else
  #endif
  {
    // Send a byte from the buffer
    UDR0 = tx_buffer[tail];

    // Update tail position
    tail++;
    if (tail == TX_BUFFER_SIZE) { tail = 0; }

    tx_buffer_tail = tail;
  }

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}

//从串口读出数据
uint8_t serial_read()
{
  if (rx_buffer_head == rx_buffer_tail)
  {
    return SERIAL_NO_DATA;
  }
  else
  {
    uint8_t data = rx_buffer[rx_buffer_tail];
    rx_buffer_tail++;
    if (rx_buffer_tail == RX_BUFFER_SIZE)
    {
      rx_buffer_tail = 0;
    }

    #ifdef ENABLE_XONXOFF
      if ((get_rx_buffer_count() < RX_BUFFER_LOW) && flow_ctrl == XOFF_SENT)
      {
        flow_ctrl = SEND_XON;
        UCSR0B |=  (1 << UDRIE0); // Force TX
      }
    #endif

    return data;
  }
}

#if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega2560__)
ISR(USART0_RX_vect)
#else
ISR(USART_RX_vect)
#endif
{
  uint8_t data = UDR0;
  uint8_t next_head;

  // Pick off runtime command characters directly from the serial stream. These characters are
  // not passed into the buffer, but these set system state flag bits for runtime execution.
  switch (data) {
    case CMD_STATUS_REPORT: sys.execute |= EXEC_STATUS_REPORT; break; // Set as true
    case CMD_CYCLE_START:   sys.execute |= EXEC_CYCLE_START; break; // Set as true
    case CMD_FEED_HOLD:     sys.execute |= EXEC_FEED_HOLD; break; // Set as true
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    default: // Write character to buffer
      next_head = rx_buffer_head + 1;
      if (next_head == RX_BUFFER_SIZE) { next_head = 0; }

      // Write data to buffer unless it is full.
      if (next_head != rx_buffer_tail) {
        rx_buffer[rx_buffer_head] = data;
        rx_buffer_head = next_head;

        #ifdef ENABLE_XONXOFF
          if ((get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT) {
            flow_ctrl = SEND_XOFF;
            UCSR0B |=  (1 << UDRIE0); // Force TX
          }
        #endif

      }

  }
}

//清空读缓存
void serial_reset_read_buffer()
{
  rx_buffer_tail = rx_buffer_head;

  #ifdef ENABLE_XONXOFF
    flow_ctrl = XON_SENT;
  #endif
}
