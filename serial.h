//serial.c - Low level functions for sending and recieving bytes via the serial port

#ifndef serial_h
//标志头文件格式，防止 头文件 被重复引用。
#define serial_h

//nuts_bolts.h - Header file for shared definitions, variables, and functions.共享 定义、变量和函数
#include "nuts_bolts.h"

//定义接收缓存 为 128
#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
//定义发送缓存 为 64
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 64
#endif

//串口无数据
#define SERIAL_NO_DATA 0xff

//不知道怎么用？ 也不知道 watermark 水印 是什么意思？
#ifdef ENABLE_XONXOFF
  #define RX_BUFFER_FULL 96 // XOFF high watermark
  #define RX_BUFFER_LOW 64 // XON low watermark
  #define SEND_XOFF 1
  #define SEND_XON 2
  #define XOFF_SENT 3
  #define XON_SENT 4
  #define XOFF_CHAR 0x13
  #define XON_CHAR 0x11
#endif

//串口初始化函数
void serial_init();

//将数据 data 写入串口
void serial_write(uint8_t data);

//从串口读出数据
uint8_t serial_read();

//清空读缓存
// Reset and empty data in read buffer. Used by e-stop and reset.
void serial_reset_read_buffer();

#endif
