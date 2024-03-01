# MRS Project: Ultrasound Test (Prototype)
HW &amp; SW Project for feasibility assessment of accoustic localisation on board of UAVs

## HW Notes
* Project for Altium Designer 24.1.2
* Board power: +5V, fed to regulator AMS1117-3.3 with 3.3V output
* Main MCU: [STM32G030F6P6](https://www.st.com/en/microcontrollers-microprocessors/stm32g030f6.html) (or G031) running at 64MHz (PLL from HSI)
* MEMS microphone: ST [IMP23ABSU](https://www.st.com/en/mems-and-sensors/imp23absu.html) with 80kHz bandwidth ([STEVAL-MIC007V1](https://www.st.com/en/evaluation-tools/steval-mic007v1.html))
* Analog circuitry: PGA [MAX9939AUB+](https://www.analog.com/en/products/max9939.html), offers gain values 0.25 to 157 + optional multi-feedback filter (LP/HP/BP) on uncommited op-amp
* Selectable order of PGA and MFB filter (solder bridges SB2, SB4 closed for PGA -> MFB / SB1, SB3 for MFB -> PGA)
* Data output: USART1 (optionally with CTS flow control pin = SYNC pin); used with USB-UART adapter supporting baudrates >3Mbps (e.g., CH343)
* Exposed SWD programming interface (standard Nucleo ST-Link pin order)

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
* ADC data is transferred by DMA to RAM buffer (with size 2kB), first and second half of the buffer are processed separately in the main program loop: the ADC data is 12-bit unsigned (0-4095) stored by default in 16-bit unsigned, the main loop "compresses" every two 12-bit consequtive samples into three bytes (saves 25% of the required throughput); sampling rate 160kHz thus corresponds to only 240kBps sample data throughput
* The output data packets are transmitted via USART1 using another DMA channel, packets have total size 764B and consist of:
  - 5 delimeter bytes (0xFF) for packet start alignment,
  - 1 byte for selected PGA gain (uint8, values 0 to 9) + 1 byte for selected PGA offset (int8, values -15 to +15) + 1 byte for selected ADC channel (uint8, values 0 to 2),
  - 4 bytes for incremental packet ID (uint32),
  - 750 bytes for the 500 compressed 16-bit samples,
  - 2 dummy bytes for packet size alignment to 32-bit multiples
* Accepts USART commands for settings management:
  - `S0`/`S1` for PGA shutdown disable/enable,
  - `M0`/`M1` for PGA self-measurement disable/enable,
  - `G0` to `G9` for PGA gain selection (corresponding to 1, 10, 20, 30, 40, 60, 80, 120, 157, 0.25 gains),
  - `O+0`/`O-0` to `O+15`/`O-15` for internal PGA offset correction (+/- signed, corresponding to 0.0, 1.3, 2.5, 3.8, 4.9, 6.1, 7.3, 8.4, 10.6, 11.7, 12.7, 13.7, 14.7, 15.7, 16.7, 17.6 mV),
  - `C0` to `C2` for ADC channel selection (CH0, CH1 or CH2)

## SW Notes
* Python3 script which uses Tkinter + matplotlib for real-time visualisation of the ADC measurement: shows signal + FFT amplitude spectrum, provides simple GUI for selection of PGA gain, PGA offset and ADC channel
* Python ROS1 node which unpacks and publishes topic with the raw ADC data, automatically adjusts the PGA gain in case of saturation of the sampled analog signal
