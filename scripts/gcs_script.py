import driver
from pymavlink import mavutil

gc = driver.GroundControlChannel(device='/dev/ttyUSB1', baud=57600)

gc.wait_heartbeat()
print("rx hb")
while 1:
    msg = gc.recv_match()
    if msg != None:
        print(msg)