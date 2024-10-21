import serial
import RPi.GPIO as GPIO
import time
import struct

# UART and RS485 Configuration
UART_PORT = '/dev/ttyS0'  # Hardware UART port
BAUD_RATE = 9600
TIMEOUT = 1
DERE_PIN = 4  # GPIO4 for Data Enable/Receive Enable

# Anemometer Parameters
DEFAULT_DEVICE_ADDRESS = 0x03 # Device address can be different, please look at the datasheet
WIND_SPEED_REG_ADDR = 0x002A
READ_HOLDING_REG = 0x03

def calculate_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def get_wind_speed(address, serial_con):
    request = struct.pack('>BBHH', address, READ_HOLDING_REG, WIND_SPEED_REG_ADDR, 0x0001)
    crc = calculate_crc(request)
    request += struct.pack('<H', crc)
    GPIO.output(DERE_PIN, GPIO.HIGH)  # Enable transmit
    serial_con.write(request)
    serial_con.flush()
    GPIO.output(DERE_PIN, GPIO.LOW)  # Enable receive

    response = serial_con.read(7)

    print(response.hex())
    if len(response) != 7:
        raise Exception("Invalid response length")

    wind_speed = struct.unpack('>H', response[3:5])[0] / 100.0
    return wind_speed

def main():
    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DERE_PIN, GPIO.OUT)
    GPIO.output(DERE_PIN, GPIO.LOW)  # Initially set to receive mode

    # Initialize serial connection
    serial_con = serial.Serial(
        port=UART_PORT,
        baudrate=BAUD_RATE,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=TIMEOUT
    )

    try:
        while True:
            try:
                wind_speed = get_wind_speed(DEFAULT_DEVICE_ADDRESS, serial_con)
                print(f"Wind speed: {wind_speed:.2f} m/s")
            except Exception as e:
                print(f"Error reading wind speed: {e}")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Program terminated by user")

    finally:
        serial_con.close()
        GPIO.cleanup()
        print("Serial port closed and GPIO cleaned up")

if __name__=="__main__":
    main()
