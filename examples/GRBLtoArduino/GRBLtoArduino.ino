/*
  Thanks for supporting Open-Hard/Soft-ware and thanks
  for all of the contributors to this project.

  For extra info on GRBL please have a look at my blog :
	http://blog.protoneer.co.nz/tag/grbl/

																	--> spindle_control
	serial --> protocol --> gcode --
																	--> motion_control --> planner --> stepper

	Supported hardware:
		Arduino Duemilanove
		Arduino Uno
		Arduino Mega 2560 (Limited Testing)

1.'protocol'        :接收串口命令传递给gcode执行，给命令提供应答，通过串口中断管理程序命令
2.'gcode'           :从1接收G代码，解析G代码
3.'spindle_control' :主轴控制，雕刻机的主轴带刀的轴，与XYZ无关，M3,4,5有关于主轴正反转停止命令
4.'motion_control'  :从2接收运动命令将其传递给5，相当于一个发出命令的高层接口
5.'planner'         :从4接收线性运动的命令，并将其添加到准备运动计划中(计算的数据写入唤醒缓冲区)
                    随着运动不断被添加负责优化计算加速度分布图
6.'stepper'         :执行动作，用两个定时器来控制三个轴完成相应的动作

7.Supporting files:
'config.h'        :一些全局变量的宏定义，例如MINIMUM_STEPS_PER_MINUTE最低每分钟跳动多少下，也就是最低 
                   频率就在这里声明的
'settings'        :全局中主要的参数设置，$$命令打印出来的参数都是这里设置的
                   上电读取EEPROM的值，如果读取失败调用default值，都在这里
'eeprom'          :存放参数的作用，有关于EPROM的读写函数
'nuts_bolts.h'    :一些全局变量的定义
'serial'          :串口控制台
'print'           :打印不同格式的字符串函数在这里定义的
*/

#include <grblmain.h>

void setup(){
	startGrbl();
}

void loop(){}
