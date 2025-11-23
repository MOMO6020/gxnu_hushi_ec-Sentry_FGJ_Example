# GXNU虎师队电控框架2025赛季

### 介绍
本项目提供了可供后续开发机器人的框架性代码，在程序编写的过程中参考了[跃鹿战队的代码框架](https://gitee.com/hnuyuelurm/basic_framework),与之不同的是，我们的这个框架是基于C++编写的,我们也非常建议刚刚进入队伍的萌新去阅读一下他们文档，你或许会对整个比赛以及开发流程有更详细的了解。
同时请注意，由于开发这个框架的时候我们这一届的开发者已经要毕业了，所以我们目前的代码框架任然存在着许多的不足之处以及未完善的功能，期待着大家共同努力来完善我们目前的框架。

### 软件架构
软件架构说明
```
.
├─.vscode                   # VSCode 工程配置目录，如编译参数、调试配置等
├─app                       # 用户应用层代码，主要包括主函数、任务创建等顶层逻辑
├─bsp                       # Board Support Package，板级支持包
│  ├─can                    # 与 CAN 总线相关的驱动和初始化代码
│  ├─log                    # 日志系统模块，用于调试输出
│  ├─structure              # 通用数据结构
│  ├─system                 # 系统工具模块，如 DWT 计时、互斥锁、时间管理等
│  ├─uart                   # UART 串口通信驱动
│  └─usb                    # USB虚拟串口驱动   (可能存在问题 TODO)
├─modules                   # 功能模块目录(业务层封装，各子系统模块)
│  ├─buzzer                 # 蜂鸣器控制模块    (未实现)
│  ├─controller             # 控制器模块，如遥控器、手柄等输入解析
│  ├─daemon                 # 守护进程模块，监控系统运行状态
│  ├─imu                    # 惯性测量单元模块，姿态角度传感器驱动
│  ├─motor                  # 电机模块 (后续考虑抽象不同品牌电机接口)
│  │  └─DJIMotor            # DJI 电机专用驱动封装
│  ├─pid                    # PID 控制模块
│  ├─umt                    # 移植自上交开源的线程间通信模块，后续考虑重新实现
│  └─vision                 # 视觉模块，处理上位机回传数据(视觉部分未经测试 TODO)
├─examples                  # 示例代码、测试用例等
│  └─Serial_motor           # 电机串口调试示例
├─Middlewares               # 第三方中间件库
│  ├─ST                     # ST 官方中间件(如HAL库、USB库等)
│  └─Third_Party            # 第三方组件
│      ├─FreeRTOS           # 实时操作系统 FreeRTOS 源码
│      └─SEGGER             # SEGGER RTT/RTT Viewer 支持库
├─cmake                     # CMake 相关脚本
│  └─stm32cubemx            # STM32CubeMX 生成的 CMake 配置文件目录
├─Drivers                   # STM32 HAL 驱动库 (由 CubeMX 自动生成 通常无需修改)
├─Inc                       # 工程头文件目录 (由 CubeMX 自动生成 通常无需修改)
├─Src                       # 工程源文件目录 (由 CubeMX 自动生成 通常无需修改)
├─CMakeLists.txt            # 顶层 CMake 构建脚本
├─openocd_dap.cfg           # OpenOCD 调试配置(DAPLink 使用)
├─openocd_stlink.cfg        # OpenOCD 调试配置(ST-Link 使用)
├─startup_stm32f407xx.s     # 启动汇编文件，由 STM32 启动时执行
├─STM32F407.svd             # SVD 文件，用于调试器显示寄存器信息
├─STM32F407XX_FLASH.ld      # 链接脚本文件，定义 Flash 与 RAM 地址分布
├─README.md                 # 本文档
├─LICENSE                   # 开源许可文件
├─.gitignore                # Git 忽略规则配置文件
└─.gitattributes            # Git 属性配置文件(控制换行符、差异合并策略等)
``` 

#### 安装教程

详细的安装和配置教程待完善，在完善之前可以参考我们的Doc文件进行配置


