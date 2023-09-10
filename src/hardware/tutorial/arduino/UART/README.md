# UART communication setup between Arduino and Raspberry Pi

In this tutorial, we are going to use UART communication protocol for serial communication between the two devices. 

## Connecting Arduino UNO and raspberry Pi

There are several ways in which they can be connected. 

### Method 1. Direction connection with USB

![USB connection](https://github.com/Shaswat2001/quadrotor_raspberry_pi/blob/main/resources/raspberrypi_arduino_uno_uart_usb.png?raw=true)

### Method 2. Serial connection using GPIO

![GPIO connection](https://github.com/Shaswat2001/quadrotor_raspberry_pi/blob/main/resources/raspberrypi_arduino_uart.png?raw=true)

Since, arduino UNO operates at 5V and raspberry pi operates at 3.3V, a **3.3V/5V level-shifter** is needed to protect the raspberry. 

If the arduino is connected using USB, the board should be detected as ```/dev/ttyACM0``` and if connected using GPIO, it should appear as ```/dev/ttyS0```. Detailed explaination of UART communication over GPIOs can be found in the <a href="https://github.com/Shaswat2001/quadrotor_raspberry_pi/blob/main/src/hardware/gps/README.md">GPS tutorial</a>.

For now, we will be testing the serial communication using USB. 

## Communication code 

Upload the code ```arduino_uart.ino``` in the arduino UNO. Once done, connect raspberry with arduino UNO using USB. To test the serial communication, run ```raspberry.py``` on Pi. If the communication is successful, you should have ```You sent me: Hello from Raspberry Pi!``` as output. 

