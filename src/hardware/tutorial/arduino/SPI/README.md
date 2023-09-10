# SPI communication setup between Arduino and Raspberry Pi

In this tutorial, we are going to use SPI communication protocol for serial communication between the two devices. 

## Connecting Arduino UNO and raspberry Pi

For SPI communication, the following connection should be made 

![SPI connection](https://github.com/Shaswat2001/quadrotor_raspberry_pi/blob/main/resources/rpi_arduino_spi.png?raw=true)

If Raspberry Pi is configured as master and ardunio UNO is configured as slave, then **3.3V/5V level-shifter** is not needed. However, if the configuration is switched, the level-shifter is needed to prevent few GPIOs from burning. 

In SPI, the following abbreviations are used - 
* MISO (Master In Slave Out)
* MOSI (Master Out Slave In)

With Pi as master, the following connections are made - 
* Link the GND of the Raspberry Pi to the GND of the Arduino.
* Connect the MISO of the Pi (pin 19) to the Arduino MISO (pin 12).
* Connect the MOSI of the Pi (pin 21) to the Arduino MOSI (pin 11).

## Enabling I2C communication protocol

The first step is to enable I2C in the configuration file. To do this, edit the ```/boot/firmware/config.txt``` file using the below command - 

```
sudo nano /boot/firmware/config.txt
```

In the above file, there is a line ```#dtparam=spi=on```. Meaning it is currently inactive. To make it active, remove ```#```.

Then just reboot the raspberry pi - 

```
sudo reboot
```

