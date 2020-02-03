//protocol.c - the serial protocol master control unit
//接收串口命令传递给gcode执行，给命令提供应答，通过串口中断管理程序命令
#include <avr/io.h>
#include <avr/interrupt.h>
#include "protocol.h"
#include "gcode.h"          //rs274/ngc parser
#include "serial.h"         //
#include "print.h"          //Functions for formatting output strings
#include "settings.h"       //eeprom configuration handling
#include "config.h"
#include "nuts_bolts.h"     //Header file for shared definitions, variables, and functions
#include "stepper.h"        //stepper motor driver: executes motion plans of planner.c using the stepper motors
#include "report.h"         //reporting and messaging methods
#include "motion_control.h" //high level interface for issuing motion commands

/*
为什么 静态？ http://bbs.csdn.net/topics/350238100 7楼
定义静态函数的好处：
<1> 其他文件中可以定义相同名字的函数，不会发生冲突
<2> 静态函数不能被其他文件所用。

同样也可以理解 为什么 char_counter 静态变量

由于static变量的以上特性，可实现一些特定功能。
1． 统计次数功能
声明函数的一个局部变量，并设为static类型，作为一个计数器，这样函数每次被调用的时候就可以进行计数。
这是统计函数被调用次数的最好的办法，因为这个变量是和函数息息相关的，而函数可能在多个不同的地方被调用，所以从调用者的角度来统计比较困难。
*/

static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated. 要执行的行。 零时终止。
static uint8_t char_counter; // Last character counter in line variable. 行变量中的最后一个字符计数器。
static uint8_t iscomment; // Comment/block delete flag for processor to ignore comment characters. 注释/块删除标志，供处理器忽略注释字符。

//初始化 protocol
void protocol_init()
{
  char_counter = 0; // Reset line input 复位
  iscomment = false;
  report_init_message(); // Welcome message 欢迎信息

  //设置端口

  // Set as input pins对 PINOUT_MASK
  // 取反，就是 arduino上 A0 A1 A2 端口取0
  PINOUT_DDR &= ~(PINOUT_MASK);
  // Enable internal pull-up resistors. Normal high operation.
  //
  PINOUT_PORT |= PINOUT_MASK;
  // Enable specific pins of the Pin Change Interrupt
  // A0 A1 A2 端口可以中断。
  PINOUT_PCMSK |= PINOUT_MASK;
  // Enable Pin Change Interrupt
  // 这里需要 理解 AVR 中断机制。暂时理解为 设置哪些引脚可以中断。
  PCICR |= (1 << PINOUT_INT);
}

// Executes user startup script, if stored.
// N_STARTUP_LINE 定义 在 config.h 中， #define N_STARTUP_LINE 2
void protocol_execute_startup()
{
  uint8_t n;
  for (n=0; n < N_STARTUP_LINE; n++)
  {
    /*其中，settings_read_startup_line 函数， 定义在 setting.c 文件中，
    Reads startup line from EEPROM. Updated pointed line string data.
    没有成功，返回 false*/
    if (!(settings_read_startup_line(n, line)))
    {
      report_status_message(STATUS_SETTING_READ_FAIL);
    }
    else
    {
      if (line[0] != 0)
      {
        //printString 函数 定义在print.c 中，串口写入
        printString(line); // Echo startup line to indicate execution. 回显启动行以指示执行。
        report_status_message(gc_execute_line(line));
      }
    }
  }
}

// Pin change interrupt for pin-out commands, i.e. cycle start, feed hold, and reset. Sets
// only the runtime command execute variable to have the main program execute these when
// its ready. This works exactly like the character-based runtime commands when picked off
// directly from the incoming serial data stream.

//we’ll need to tell the AVR which pins we’d like for it to watch specifically.
//This is done through the pin mask, which is just a normal 8-bit byte where the corresponding bit is set for each pin we’d like to be able to trigger the interrupt.
/*
引脚输出命令的引脚更改中断，即循环启动，进给保持和复位。
仅设置运行时命令执行变量，以使主程序在就绪时执行它们。
完全像拾取基于字符的运行时命令一样工作直接从传入的串行数据流。
*/
//#define PINOUT_INT_vect  PCINT1_vect 这里要再 学习 一下 AVR 的中断，PC0 through PC6 trigger PCINT1.
ISR(PINOUT_INT_vect)
{
  // Enter only if any pinout pin is actively low.
  if ((PINOUT_PIN & PINOUT_MASK) ^ PINOUT_MASK)//判断条件感觉有点复杂，PINOUT_PIN先和PINOUT_MASK 按位与，然后再 按位 异或。只有 PINOUT_PIN是 低电平，才能为真
  {
    if (bit_isfalse(PINOUT_PIN,bit(PIN_RESET)))//bit_isfalse 是 个宏，定义在 nuts_bolts.h中
    {
      mc_reset();//mc_reset 定义在 motion_control.c 中，system reset
    }
    else if (bit_isfalse(PINOUT_PIN,bit(PIN_FEED_HOLD)))
    {
      sys.execute |= EXEC_FEED_HOLD;//sys 一个复杂的结构体，在nuts_bolts.h中定义 // bitmask 00001000
    }
    else if (bit_isfalse(PINOUT_PIN,bit(PIN_CYCLE_START)))
    {
      sys.execute |= EXEC_CYCLE_START; // bitmask 00000010
    }
  }
}

// Executes run-time commands, when required. This is called from various check points in the main
// program, primarily where there may be a while loop waiting for a buffer to clear space or any
// point where the execution time from the last check point may be more than a fraction of a second.
// This is a way to execute runtime commands asynchronously (aka multitasking) with grbl's g-code
// parsing and planning functions. This function also serves as an interface for the interrupts to
// set the system runtime flags, where only the main program handles them, removing the need to
// define more computationally-expensive volatile variables. This also provides a controlled way to
// execute certain tasks without having two or more instances of the same task, such as the planner
// recalculating the buffer upon a feedhold or override.
// NOTE: The sys.execute variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
//在需要时执行运行时命令。 这是从主程序中的各个检查点调用的，主要是在可能有while循环等待缓冲区清除空间的地方，或者从最后一个检查点开始执行时间可能超过一秒的任何点。
//这是通过grbl的g代码解析和计划功能异步执行（也称为多任务）运行时命令的方法。 此函数还用作中断以设置系统运行时标志的接口，只有主程序可以在其中处理这些标志，从而无需定义更多计算上昂贵的volatile变量。
//这也提供了一种可控制的方式来执行某些任务，而无需具有同一任务的两个或多个实例，例如计划程序在某个供料保持或超驰时重新计算缓冲区。
//注意：sys.execute变量标志由任何进程，步进或串行中断，管脚，限位开关或主程序设置。
void protocol_execute_runtime()
{
  if (sys.execute) { // Enter only if any bit flag is true 仅当任何位标志为真时才输入
    uint8_t rt_exec = sys.execute; // Avoid calling volatile multiple times 避免多次调用volatile

    // System alarm. Everything has shutdown by something that has gone severely wrong. Report
    // the source of the error to the user. If critical, Grbl disables by entering an infinite
    // loop until system reset/abort.
    if (rt_exec & (EXEC_ALARM | EXEC_CRIT_EVENT))
    {
      sys.state = STATE_ALARM; // Set system alarm state // bitmask 00100000

      // Critical event. Only hard limit qualifies. Update this as new critical events surface.
      if (rt_exec & EXEC_CRIT_EVENT)
      {
        report_alarm_message(ALARM_HARD_LIMIT);
        report_feedback_message(MESSAGE_CRITICAL_EVENT);
        bit_false(sys.execute,EXEC_RESET); // Disable any existing reset
        do {
          // Nothing. Block EVERYTHING until user issues reset or power cycles. Hard limits
          // typically occur while unattended or not paying attention. Gives the user time
          // to do what is needed before resetting, like killing the incoming stream.
        } while (bit_isfalse(sys.execute,EXEC_RESET));

      // Standard alarm event. Only abort during motion qualifies.
      }
      else
      {
        // Runtime abort command issued during a cycle, feed hold, or homing cycle. Message the
        // user that position may have been lost and set alarm state to enable the alarm lockout
        // to indicate the possible severity of the problem.
        report_alarm_message(ALARM_ABORT_CYCLE);
      }
      bit_false(sys.execute,(EXEC_ALARM | EXEC_CRIT_EVENT));
    }

    // Execute system abort.
    if (rt_exec & EXEC_RESET)
    {
      sys.abort = true;  // Only place this is set true.
      return; // Nothing else to do but exit.
    }

    // Execute and serial print status
    if (rt_exec & EXEC_STATUS_REPORT)
    {
      report_realtime_status();
      bit_false(sys.execute,EXEC_STATUS_REPORT);
    }

    // Initiate stepper feed hold
    if (rt_exec & EXEC_FEED_HOLD)
    {
      st_feed_hold(); // Initiate feed hold.
      bit_false(sys.execute,EXEC_FEED_HOLD);
    }

    // Reinitializes the stepper module running state and, if a feed hold, re-plans the buffer.
    // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
    if (rt_exec & EXEC_CYCLE_STOP)
    {
      st_cycle_reinitialize();
      bit_false(sys.execute,EXEC_CYCLE_STOP);
    }

    if (rt_exec & EXEC_CYCLE_START)
    {
      st_cycle_start(); // Issue cycle start command to stepper subsystem
      if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) {
        sys.auto_start = true; // Re-enable auto start after feed hold.
      }
      bit_false(sys.execute,EXEC_CYCLE_START);
    }
  }

  // Overrides flag byte (sys.override) and execution should be installed here, since they
  // are runtime and require a direct and controlled interface to the main stepper program.
}


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also executes Grbl internal commands, such as
// settings, initiating the homing cycle, and toggling switch states. This differs from
// the runtime command module by being susceptible to when Grbl is ready to execute the
// next line during a cycle, so for switches like block delete, the switch only effects
// the lines that are processed afterward, not necessarily real-time during a cycle,
// since there are motions already stored in the buffer. However, this 'lag' should not
// be an issue, since these commands are not typically used during a cycle.
/*
从protocol_process引导并执行一行格式化的输入。 尽管大多数传入的流式g代码块都在执行，但它还会执行Grbl内部命令，
例如设置，启动归位周期和切换开关状态。 这与运行时命令模块不同，它易受Grbl准备在周期中执行下一行的时间影响，
因此对于诸如块删除之类的开关，该开关仅影响随后处理的行，而不一定在周期中是实时的 ，因为缓冲区中已经存储了动作。
但是，这种“滞后”应该不成问题，因为这些命令通常不在循环中使用。
*/
uint8_t protocol_execute_line(char *line)
{
  // Grbl internal command and parameter lines are of the form '$4=374.3' or '$' for help
  if(line[0] == '$')
  {
    uint8_t char_counter = 1;
    uint8_t helper_var = 0; // Helper variable
    float parameter, value;
    switch( line[char_counter] )//检测第二个字符
    {
      case 0 :
        report_grbl_help();
        break;
      case '$' : // Prints Grbl settings 打印grbl设置
        if ( line[++char_counter] != 0 )
        {
          return(STATUS_UNSUPPORTED_STATEMENT);
        }
        else
        {
          report_grbl_settings();
        }
        break;
      case '#' : // Print gcode parameters 打印gcode参数
        if ( line[++char_counter] != 0 )
        {
          return(STATUS_UNSUPPORTED_STATEMENT);
        }
        else
        {
          report_gcode_parameters();
        }
        break;
      case 'G' : // Prints gcode parser state 打印gcode解析器状态
        if ( line[++char_counter] != 0 )
        {
          return(STATUS_UNSUPPORTED_STATEMENT);
        }
        else
        {
          report_gcode_modes();
        }
        break;
      case 'C' : // Set check g-code mode 设置检查g码模式
        if ( line[++char_counter] != 0 )
        {
          return(STATUS_UNSUPPORTED_STATEMENT);
        }
        // Perform reset when toggling off. Check g-code mode should only work if Grbl
        // is idle and ready, regardless of alarm locks. This is mainly to keep things
        // simple and consistent.
        //切换时执行重置。检查g码模式仅在Grbl空闲且准备就绪时才起作用，而与警报锁定无关。 这主要是为了使事情保持简单和一致。
        if ( sys.state == STATE_CHECK_MODE )
        {
          mc_reset();
          report_feedback_message(MESSAGE_DISABLED);
        }
        else
        {
          if (sys.state)
          {
            return(STATUS_IDLE_ERROR);
          }
          sys.state = STATE_CHECK_MODE;
          report_feedback_message(MESSAGE_ENABLED);
        }
        break;
      case 'X' : // Disable alarm lock 禁用警报锁定
        if ( line[++char_counter] != 0 )
        {
          return(STATUS_UNSUPPORTED_STATEMENT);
        }
        if (sys.state == STATE_ALARM)
        {
          report_feedback_message(MESSAGE_ALARM_UNLOCK);
          sys.state = STATE_IDLE;
          // Don't run startup script. Prevents stored moves in startup from causing accidents.
          //不要运行启动脚本。 防止启动时存储的动作引起事故。
        }
        break;
      case 'H' : // Perform homing cycle
        if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE))
        {
          // Only perform homing if Grbl is idle or lost.
          //仅在Grbl空闲或丢失时执行归位。
          if ( sys.state==STATE_IDLE || sys.state==STATE_ALARM )
          {
            mc_go_home();
            if (!sys.abort)
            {
              protocol_execute_startup();
            } // Execute startup scripts after successful homing.成功归位后执行启动脚本。
          }
          else
          {
            return(STATUS_IDLE_ERROR);
          }
        }
        else
        {
          return(STATUS_SETTING_DISABLED);
        }
        break;
      //    case 'J' : break;  // Jogging methods
      // TODO: Here jogging can be placed for execution as a seperate subprogram. It does not need to be
      // susceptible to other runtime commands except for e-stop. The jogging function is intended to
      // be a basic toggle on/off with controlled acceleration and deceleration to prevent skipped
      // steps. The user would supply the desired feedrate, axis to move, and direction. Toggle on would
      // start motion and toggle off would initiate a deceleration to stop. One could 'feather' the
      // motion by repeatedly toggling to slow the motion to the desired location. Location data would
      // need to be updated real-time and supplied to the user through status queries.
      //   More controlled exact motions can be taken care of by inputting G0 or G1 commands, which are
      // handled by the planner. It would be possible for the jog subprogram to insert blocks into the
      // block buffer without having the planner plan them. It would need to manage de/ac-celerations
      // on its own carefully. This approach could be effective and possibly size/memory efficient.
      case 'N' : // Startup lines.
        if ( line[++char_counter] == 0 )
        { // Print startup lines
          for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++)
          {
            if (!(settings_read_startup_line(helper_var, line)))
            {
              report_status_message(STATUS_SETTING_READ_FAIL);
            }
            else
            {
              report_startup_line(helper_var,line);
            }
          }
          break;
        }
        else
        { // Store startup line
          helper_var = true;  // Set helper_var to flag storing method.
          // No break. Continues into default: to read remaining command characters.
        }
      default :  // Storing setting methods
        if(!read_float(line, &char_counter, &parameter))
        {
          return(STATUS_BAD_NUMBER_FORMAT);
        }
        if(line[char_counter++] != '=')
        {
          return(STATUS_UNSUPPORTED_STATEMENT);
        }
        if (helper_var)
        { // Store startup line
          // Prepare sending gcode block to gcode parser by shifting all characters
          helper_var = char_counter; // Set helper variable as counter to start of gcode block
          do
          {
            line[char_counter-helper_var] = line[char_counter];
          } while (line[char_counter++] != 0);
          // Execute gcode block to ensure block is valid.
          helper_var = gc_execute_line(line); // Set helper_var to returned status code.
          if (helper_var)
          {
            return(helper_var);
          }
          else
          {
            helper_var = trunc(parameter); // Set helper_var to int value of parameter
            settings_store_startup_line(helper_var,line);
          }
        }
        else
        { // Store global setting.
          if(!read_float(line, &char_counter, &value))
          {
            return(STATUS_BAD_NUMBER_FORMAT);
          }
          if(line[char_counter] != 0)
          {
            return(STATUS_UNSUPPORTED_STATEMENT);
          }
          return(settings_store_global_setting(parameter, value));
        }
    }
    return(STATUS_OK); // If '$' command makes it to here, then everything's ok.如果“ $”命令到达此处，则一切正常。
  }
  else
  {
    return(gc_execute_line(line));    // Everything else is gcode 其它类型的串口字符输入都应该是gcode
  }
}


// Process and report status one line of incoming serial data. Performs an initial filtering
// by removing spaces and comments and capitalizing all letters.
//处理和报告状态一行输入的串行数据。 通过删除空格和注释并大写所有字母来执行初始过滤。
void protocol_process()
{
  uint8_t c;
  while((c = serial_read()) != SERIAL_NO_DATA)
  {
    if ((c == '\n') || (c == '\r')) { // End of line reached

      // Runtime command check point before executing line. Prevent any furthur line executions.
      // NOTE: If there is no line, this function should quickly return to the main program when
      // the buffer empties of non-executable data.
      //执行代码行之前的运行时命令检查点。 防止执行任何进一步的行动。
      //注意：如果没有任何行，则当缓冲区清空不可执行的数据时，此函数应快速返回主程序。
      protocol_execute_runtime();
      if (sys.abort) { return; } // Bail to main program upon system abort 系统中止后对主程序进行释放

      if (char_counter > 0) // Line is complete. Then execute! 行是完整的。 然后执行！
      {
        line[char_counter] = 0; // Terminate string 字符计数器清零
        report_status_message(protocol_execute_line(line));
      }
      else
      {
        // Empty or comment line. Skip block.//为空或注释行。 跳过块。
        report_status_message(STATUS_OK); // Send status message for syncing purposes.发送状态消息以进行同步。
      }
      char_counter = 0; // Reset line buffer index
      iscomment = false; // Reset comment flag

    }
    else
    {
      if (iscomment)
      {
        // Throw away all comment characters 扔掉所有注释字符
        if (c == ')') {
          // End of comment. Resume line. 评论结束。 恢复行。
          iscomment = false;
        }
      }
      else
      {
        if (c <= ' ')
        {
          // Throw away whitepace and control characters 丢掉空格和控制字符
        }
        else if (c == '/')
        {
          // Block delete not supported. Ignore character.
        }
        else if (c == '(')
        {
          // Enable comments flag and ignore all characters until ')' or EOL.
          iscomment = true;
        }
        else if (char_counter >= LINE_BUFFER_SIZE-1)
        {
          // Throw away any characters beyond the end of the line buffer
        }
        else if (c >= 'a' && c <= 'z')
        { // Upcase lowercase
          line[char_counter++] = c-'a'+'A';
        }
        else
        {
          line[char_counter++] = c;
        }
      }
    }
  }
}
