For this demo, we are using NEO-7M satellite positoning module GPS. To communicate with the GPS, we have to follow the following steps - 

## Enabling UART communication protocol

The first step is to enable UART in the configuration file. To do this, edit the ```/boot/firmware/config.txt``` file using the below command - 

```
sudo nano /boot/firmware/config.txt
```

In the above file, there is a line ```#enable_uart=1```. Meaning it is currently inactive. To make it active, remove ```#``` and set the ```enable_uart``` variable to ```1``` to make it active and ```0``` to disable it. Once done, save the file. 

The next step is to disable certain parameters from ```/boot/firmware/cmdline.txt```, which is loaded during boot time of ubuntu. Run - 

```
sudo nano /boot/firmware/cmdline.txt
```

From the file, remove ```console=serial0,115200``` parameter (This parameter configures the OS to output messages at baud rate of 115200 to the serial console).

Following that, we need to disable the serial-getty service. To disable it, run the follwing commands in the terminal, where ```<tty_service>``` is either ```ttys0``` or ```ttyAMA0```. 

```
sudo systemctl stop serial-getty@<tty_service>.service
sudo systemctl disable serial-getty@<tty_service>.service
sudo systemctl mask serial-getty@<tty_service>.service
```

Then enable the ```ttys0``` service, 

```
sudo systemctl enable serial-getty@ttys0.service
```

Finally, to access the serial port, your user needs to have the necessary permissions. Run the following commands, replacing ```<your_username>``` with your actual username:

```
sudo adduser <your_username> tty
sudo adduser <your_username> dialout
```

Then just reboot the raspberry pi - 

```
sudo reboot
```

## Connecting GPS module to raspberry PI

Make the following connections between GPS and raspberry PI -
* VCC to Pin 1, which is 3.3V
* TX to Pin 10, which is RX (GPIO15)
* RX to Pin 8, which is TX (GPIO14)
* Gnd to Pin 6, which is ground

To test the connection, run the ```gps.py``` script. 


