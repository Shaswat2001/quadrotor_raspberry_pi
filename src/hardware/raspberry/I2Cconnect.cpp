#include <I2Cconnect.h>

I2C::I2C(int handle)
{
    ard_address = "/dev/i2c-1";
    ard_handle = open(ard_address,O_RDWR);

    if (ard_handle < 0)
    {
        std::cout<"Can't open I2C bus"<<std::endl;
    }
    
    if (ioctl(ard_handle,I2C_SLAVE,handle))
    {
        std::cout << "Can't set the I2C address for the slave device" << std::endl;
    }
}

I2C::~I2C()
{
        if (ard_handle){ //If the I2C File handle is still open...
            close(ard_handle); //...Close it.
        }
}

I2C::readInt()
{   
    const int arr_length = 2;
    char bytes[arr_length];

    int val = -1;

    if (readBytes(bytes,arr_length) > 0)
    {
        val = bytes[1] << 8 | bytes[0];
    }

    return val;
}

I2C::writeInt(int data)
{
    const int arr_length = 2;
    char bytes[arr_length];
    bytes[0] = data;
    bytes[1] = data >> 8;

    int val = (readBytes(bytes,arr_length) > 0);

    return val;
}

I2C::readBytes(char data,int length)
{
    int val =  read(ard_handle,data,length);
    return val;
}

I2C::writeBytes(char data,int length)
{
    int val =  write(ard_handle,data,length);   
    return val;
}