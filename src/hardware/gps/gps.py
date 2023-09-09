import serial
from time import sleep
import sys

def get_gps_info(GPS_buff):

    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = GPS_buff[0]                    #extract time from GPGGA string
    nmea_latitude = GPS_buff[1]                #extract latitude from GPGGA string
    nmea_longitude = GPS_buff[3]               #extract longitude from GPGGA string
    
    print("NMEA Time: ", nmea_time,'\n')
    print ("NMEA Latitude:", nmea_latitude,"NMEA Longitude:", nmea_longitude,'\n')
    
    lat = float(nmea_latitude)                  #convert string into float for calculation
    longi = float(nmea_longitude)               #convertr string into float for calculation
    
    lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
    long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format

    return lat_in_degrees, long_in_degrees
    
#convert raw NMEA string into degree decimal format   
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position


def main():

    gpgga_info = "$GPGGA"
    srl = serial.Serial("/dev/ttyS0")
    GPGGA_buffer = 0
    GPS_buff = 0
    lat_in_degrees = 0
    long_in_degrees = 0

    try:
        while True:

            gps_data = (str)(srl.readline())
            GPGGA_data_available = gps_data.find(gpgga_info)
            if (GPGGA_data_available > 0):
                GPGGA_buffer  = gps_data.split(gpgga_info,1)[1]
                GPS_buff = (GPGGA_buffer.split(','))
                print(GPS_buff)
                lat_in_degrees,long_in_degrees = get_gps_info(GPS_buff)
                print("lat in degrees:", lat_in_degrees," long in degree: ", long_in_degrees, '\n')
    
    except:
        print("ERROR")
        sys.exit(0)



if __name__ == "__main__":
    main()
