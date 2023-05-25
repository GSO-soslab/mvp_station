import driver
import time

vx = driver.VxChannel(device='/dev/ttyUSB0', baud=57600)
while 1:
    vx.send_hearbeat()
    time.sleep(0.5)
    # vx.send_gps(latitude=41425692,longitude = -71641809, depth=3)
    vx.send_battery()