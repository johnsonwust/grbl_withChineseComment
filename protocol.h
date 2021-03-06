//protocol.h - the serial protocol master control unit

#ifndef protocol_h
#define protocol_h

#include <avr/sleep.h>

// Line buffer size from the serial input stream to be executed.
// NOTE: Not a problem except for extreme cases, but the line buffer size can be too small
// and g-code blocks can get truncated. Officially, the g-code standards support up to 256
// characters. In future versions, this will be increased, when we know how much extra
// memory space we can invest into here or we re-write the g-code parser not to have his
// buffer.
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 50
#endif

// Initialize the serial protocol
void protocol_init();

// Read command lines from the serial port and execute them as they
// come in. Blocks until the serial buffer is emptied.
void protocol_process();

// Executes one line of input according to protocol
uint8_t protocol_execute_line(char *line);

// Checks and executes a runtime command at various stop points in main program
void protocol_execute_runtime();

// Execute the startup script lines stored in EEPROM upon initialization
void protocol_execute_startup();

#endif
