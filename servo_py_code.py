import time
import smbus

DEVICE_BUS = 1
DEVICE_ADDR = 0x40
SCALE = 0x79 # = 25Mhz / (4096*50) - 1

#Register addreses
MODE1 = 0x00
MODE2 = 0x01
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09
PRE_SCALE = 0xFE #Writes to PRE_SCALE register are blocked when SLEEP bit is logic 0 (MODE 1)

#bit masks
SLEEP_ON = 0x11
SLEEP_OFF = 0x01

#544 мкс    1520 мкс    2400 мкс          20000мкс 4.882
#112        311         491                          4096
#servo_min = 0x70
#servo_mid = 0x137
#servo_max = 0x1EB

time.sleep(3)

bus = smbus.SMBus(DEVICE_BUS)
bus.write_byte_data(DEVICE_ADDR, MODE1, SLEEP_ON)
bus.write_byte_data(DEVICE_ADDR, PRE_SCALE, SCALE)
bus.write_byte_data(DEVICE_ADDR, MODE1, SLEEP_OFF)


time.sleep(1)
bus.write_byte_data(DEVICE_ADDR, LED0_ON_L, 0x00)
bus.write_byte_data(DEVICE_ADDR, LED0_ON_H, 0x00)
bus.write_byte_data(DEVICE_ADDR, LED0_OFF_L, 0x37)
bus.write_byte_data(DEVICE_ADDR, LED0_OFF_H, 0x01)

time.sleep(5)
bus.write_byte_data(DEVICE_ADDR, LED0_OFF_L, 0x79)
bus.write_byte_data(DEVICE_ADDR, LED0_OFF_H, 0x00)

time.sleep(5)
bus.write_byte_data(DEVICE_ADDR, LED0_OFF_L, 0xEB)
bus.write_byte_data(DEVICE_ADDR, LED0_OFF_H, 0x01)

time.sleep(5)
bus.write_byte_data(DEVICE_ADDR, LED0_OFF_L, 0x37)
bus.write_byte_data(DEVICE_ADDR, LED0_OFF_H, 0x01)

time.sleep(5)
bus.write_byte_data(DEVICE_ADDR, LED0_OFF_L, 0x00)
bus.write_byte_data(DEVICE_ADDR, LED0_OFF_H, 0x10)