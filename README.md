# Ultrasound_Test_Prototype
HW &amp; SW Project for feasibility assessment of accoustic localisation on board of UAVs

## HW Notes
* Project for Altium Designer 24.1.2
* Board power: +5V, fed to regulator AMS1117-3.3 with 3.3V output
* Main MCU: STM32G030F6P6 (or G031) running at 64MHz (PLL from HSI)
* MEMS microphone: ST IMP23ABSU with 80kHz bandwidth
* Analog circuitry: PGA MAX9939AUB+, gain values 0.25 to 157, optional multi-feedback filter (LP/HP/BP)
* Selectable order of PGA and MFB filter (solder bridges SB2, SB4 closed for PGA -> MFB / SB1, SB3 for MFB -> PGA)
* Data output: USART1 (optionally with CTS flow control pin)
* Programming SWD pins (standard Nucleo pin order)

**3D visualisation of the board**
Top view | Bottom view
:-------------------------:|:-------------------------:
![3D_Top](/Altium/Ultrasound_Test_Prototype/3D_Top.png) | ![3D_Bottom](/Altium/Ultrasound_Test_Prototype/3D_Bottom.png)

## FW Notes
* Project for STM32CubeIDE 1.14.1
* USART1 for communication running at baudrate 4Mbps
* SPI1 for PGA control (transmit only) running at 1MHz
* ADC channels IN0 to IN2: IN0 = direct microphone output, IN1 = PGA output, IN2 = MFB op-amp output
* ADC triggered by TIM3 for sampling frequency equal to 160kHz, oversampling x8 for noise reduction
* ADC data is transmitted by DMA to local buffer, first and second half are then processed in main loop - the ADC data is 12-bit unsigned (0-4095) stored by default in 16-bit unsigned, the main loop "compresses" every two 12-bit consequtive samples into three bytes (saves 25% of the required throughput)
* The compressed samples are transferred via USART1 using another DMA channel; sampling rate 160kHz equals to 240kBps throughput
* Accepts USART commands for PGA control: S0/S1 for shutdown enable, M0/M1 for self-measurement enable, G0-G9 for gain selection (values 1, 10, 20, 30, 40, 60, 80, 120, 157, 0.25) and O+0/O-0 to O+15/O-15 for internal offset correction (+/- signed, values 0.0, 1.3, 2.5, 3.8, 4.9, 6.1, 7.3, 8.4, 10.6, 11.7, 12.7, 13.7, 14.7, 15.7, 16.7, 17.6 mV)
* Can be compiled with macro `USART_ASCII_MODE` set to `true` - then USART1 transmits single ADC sample every 500ms together with string containing PGA settings

## SW Notes
* A Python3 script which uses Tkinter + matplotlib for real-time visualisation of the ADC samples: shows signal + FFT amplitude spectrum
