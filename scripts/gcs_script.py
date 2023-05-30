#!/usr/bin/env python3
import driver
import socket


class CommsGCS:
    def __init__(self, HOST:str, PORT:str, device:str, baud:int):
        self.HOST = HOST  # The server's hostname or IP address
        self.PORT = PORT  # The port used by the server
        self.gc = driver.GroundControlChannel(device=device, baud=baud)
        self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.gc.wait_heartbeat()
        # print("rx hb")
        self.s.connect((HOST, PORT))

    def recv_match(self):
        return self.gc.recv_match()

    def encode_and_send(self, message):
        message = message.encode()
        print(message)
        self.s.sendall(message)
        # print("sent")

    def send_gps(self,msg):
        message = f"[{msg.lat}" + "," + f"{msg.lon}" + "," + f"{msg.alt}]"
        self.encode_and_send(message)

    def send_attitude(self, msg):
        message = f"[{msg.roll}" + "," + f"{msg.pitch}" + "," + f"{msg.roll}]"
        self.encode_and_send(message)


if __name__ == "__main__":
    comms = CommsGCS(HOST= "192.168.1.186", PORT=8080, device="/dev/ttyUSB1", baud=57600)
    while 1:
        msg = comms.recv_match()
        if msg != None:
            type_msg = msg.get_type()
            if type_msg == 'GPS_INPUT':
                comms.send_gps(msg)
            elif type_msg == 'ATTITUDE':
                comms.send_attitude(msg)