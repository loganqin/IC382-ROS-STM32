# STM32F1 ROS Subscriber

This repository is tested and developed using Nucleo 64 STM32F103RBT6. If you need to use other MCU, you may need to make certain changes.

## A. STM32CUBMX Config
1) You need to setup DMA for USART2
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/01_dma_rx.PNG)
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/01_dma_tx.PNG)

2) You need to enable USART2 Global Interrupt
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/02_uart_it.PNG)

3) You need to set the USART2 Baudrate
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/03_uart_baud.PNG)

4) You need to ensure the clock tress is same as the following
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/10-clock_tree.PNG)

5) You need to follow the setting below to configure the project.
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/04_project_setting.PNG)
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/05_code_gen.PNG)

## B. ROS Libraries implementation
You need to extract the given **RosLibs.zip** and put all the extracted files as shown in below.
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/06-extract_zip.PNG)

## C. uVision Config
1) Open the uvision and add one group named as **ROS**. Also, you need to import those three files properly.
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/07-add_group.PNG)

2) You need to add three lines of code in the **Application/User/Core/main.c**.
```
# include "ros_main.h" # Line 27
setup();               # Line 91
loop();                # Line 98
```

3) If you are using STM32F1, please ensure you have chosen the correct design in **RosLibs\STM32Hardware.h"
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/11-stm32hardware.PNG)

4) You need to change the config of the compiler.

For **Define** in preprocessor symbols,
```
,__USE_C99_MATH 
```
For **Misc Controls**,
```
--diag_suppress=1,47,1300
```
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/08-compiler_config.PNG)

5) You need to include all the path shown in below.
![image](https://github.com/vincent51689453/agv_base_control/blob/stm32-f1/git_image/09-add_path.PNG)