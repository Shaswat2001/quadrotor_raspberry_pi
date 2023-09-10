#ifndef I2C_CONNECT_H
#define I2C_CONNECT_H

#include <iostream>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>    // For O_RDWR
#include <unistd.h>   // For open()

class I2C
{
    public:

        I2C(int handle);
        ~I2C();
        int readInt();
        int writeInt(int data);
        int writeBytes(char data,int length);
        int readBytes(char data,int length);

    private:
        const char* ard_address;
        int ard_handle;
};

#endif