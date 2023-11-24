# NXP Application Code Hub
[<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>](https://www.nxp.com)

## AN13509：Analysis of screen pushing scene based on i.MXRT595 MIPI DSI Controller
AN13509 briefly describes the principle of drawing using GPU and MIPI DSI, and measures the power consumption and frame rate of drawing and pushing screens under different conditions.<br />
Please refer to AN13509 for complete introduction on how to use this software. https://www.nxp.com.cn/docs/en/application-note/AN13509.pdf.<br />

i.MXRT595 is a dual-core microcontroller combined a graphics engine and a streamlined Cadence® Tensilica® Fusion F1 DSP core with Arm® Cortex®- M33 core. It offers an embedded solution for smart watch or other display device due to the role of GPU.<br />

Power consumption and frame rate are the considerations for performance evaluation during pushing screen. These two points are affected by many factors, such as the number and location of frame buffer and complexity of graphics. This document illustrates graphics drawing methods and performance comparisons under different conditions.<br />


![](images/Overall%20connection.png)

This document briefly describes the principle of drawing using GPU and MIPI DSI, and measures the power consumption and frame rate of drawing and pushing screens under different conditions.<br />

#### Boards: EVK-MIMXRT595
#### Categories: Graphics
#### Peripherals: CLOCKS, DMA, GPIO, TIMER, UART
#### Toolchains: MCUXpresso IDE

## Table of Contents
1. [Software](#step1)
2. [Hardware](#step2)
3. [Setup](#step3)
4. [Results](#step4)
5. [FAQs](#step5) 
6. [Support](#step6)
7. [Release Notes](#step7)

## 1. Software<a name="step1"></a>
The software for this Application Note is delivered in raw source files and MCUXpresso projects.
Software version:
- SDK: v2.9.2 
- IDE: MCUXpresso IDE v11.4.0


## 2. Hardware<a name="step2"></a>
- Micro USB cable
- EVK-MIMXRT595 SCH-45800 REV D1 board（ https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/i-mx-rt595-evaluation-kit:MIMXRT595-EVK ）
- Personal Computer
- G1120B0MIPI smart MIPI panel（ https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/1-2-wearable-display-g1120b0mipi:G1120B0MIPI ）

<p align="center">
	<img src="https://www.nxp.com/assets/images/en/dev-board-image/I.MXRT595EVK-FRONT.jpg">
</p>

![](images/Hardware%20connection.png)

## 3. Setup<a name="step3"></a>
1. Connect MIPI panel to J44.
2. Connect a USB cable between the host PC and the OpenSDA USB port(J40) on the target board.
3. Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4. Download the program to the target board.
5. Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.


## 4. Results<a name="step4"></a>
There are two kinds of graphics to display. One is a tiger and the other is a clock. We can set different images, different frame buffer numbers, and different frame buffer locations to observe the power consumption in this scenario.
For power observation points, the 4.3 section in AN13509 gives the detailed description,please refer to it.<br />

Bekow are the simple graphics and complex graphics examples.

![](images/simple%20graphics.jpg)

![](images/complex%20graphics.png)


For the frame rate and power consumption result in the different scenario, please refer to AN13509.




## 5. FAQs<a name="step5"></a>


## 6. Support<a name="step6"></a>
#### Project Metadata
<!----- Boards ----->
[![Board badge](https://img.shields.io/badge/Board-EVK&ndash;MIMXRT595-blue)](https://github.com/search?q=org%3Anxp-appcodehub+EVK-MIMXRT595+in%3Areadme&type=Repositories)

<!----- Categories ----->
[![Category badge](https://img.shields.io/badge/Category-GRAPHICS-yellowgreen)](https://github.com/search?q=org%3Anxp-appcodehub+graphics+in%3Areadme&type=Repositories)

<!----- Peripherals ----->
[![Peripheral badge](https://img.shields.io/badge/Peripheral-CLOCKS-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+clocks+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-DMA-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+dma+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-GPIO-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+gpio+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-TIMER-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+timer+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-UART-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+uart+in%3Areadme&type=Repositories)

<!----- Toolchains ----->
[![Toolchain badge](https://img.shields.io/badge/Toolchain-MCUXPRESSO%20IDE-orange)](https://github.com/search?q=org%3Anxp-appcodehub+mcux+in%3Areadme&type=Repositories)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

>**Warning**: For more general technical questions regarding NXP Microcontrollers and the difference in expected funcionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/@NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/Twitter-Follow%20us%20on%20Twitter-white.svg)](https://twitter.com/NXP)

## 7. Release Notes<a name="step7"></a>
| Version | Description / Update                           | Date                        |
|:-------:|------------------------------------------------|----------------------------:|
| 1.0     | Initial release on Application Code HUb        | June 13<sup>th</sup> 2023 |

