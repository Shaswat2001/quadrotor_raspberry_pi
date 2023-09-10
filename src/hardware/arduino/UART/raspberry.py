import serial
import time
def main():

    srl = serial.Serial("/dev/ttyACM0")
    srl.reset_input_buffer()

    while True:
        srl.write(b"Hello from Raspberry Pi!\n")
        line = srl.readline().decode('utf-8').rstrip()
        print(line)
        time.sleep(1)


if __name__ == "__main__":

    main()