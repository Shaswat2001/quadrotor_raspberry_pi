# I2C communication setup between Arduino and Raspberry Pi

In this tutorial, we are going to use I2C communication protocol for serial communication between the two devices. 

## Connecting Arduino UNO and raspberry Pi

For I2C communication, the following connection should be made 

![I2C connection](https://github.com/Shaswat2001/quadrotor_raspberry_pi/blob/main/resources/rpi_arduino_i2c.png?raw=true)

If Raspberry Pi is configured as master and ardunio UNO is configured as slave, then **3.3V/5V level-shifter** is not needed. However, if the configuration is switched, the level-shifter is needed to prevent few GPIOs from burning. 

With Pi as master, the following connections are made - 
* Link the GND of the Raspberry Pi to the GND of the Arduino.
* Connect the SDA (I2C data) of the Pi (pin 2) to the Arduino SDA.
* Connect the SCL (I2C clock) of the Pi (pin 3) to the Arduino SCL.

## Enabling I2C communication protocol

The first step is to enable I2C in the configuration file. To do this, edit the ```/boot/firmware/config.txt``` file using the below command - 

```
sudo nano /boot/firmware/config.txt
```

In the above file, there is a line ```#dtparam=i2c_arm=on```. Meaning it is currently inactive. To make it active, remove ```#```.

Then just reboot the raspberry pi - 

```
sudo reboot
```

