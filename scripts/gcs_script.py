#!/usr/bin/env python3
import driver
import socket
import threading
import time

class CommsGCS:
    def __init__(self, HOST:str, PORT:str, device:str, baud:int):
        self.HOST = HOST  # The server's hostname or IP address
        self.PORT = PORT  # The port used by the server
        self.gc = driver.GroundControlChannel(device=device, baud=baud)
        self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.msg = None
        #Socket Server
        self.s.bind((HOST, PORT))
        self.gc.wait_heartbeat()
        print("Heartbeat Received")
        self.threadlock_speed = 0.25

        #One thread to monitor new MAVmsgs, One thread to listen for new clients and send msg through sockets.
        threading.Thread(target=self.listen).start()
        threading.Thread(target=self.recv_mavlink_msg).start()
        

    def listen(self):
        print("Starting to listen for socket client")
        self.s.listen()
        while True:
            conn, addr = self.s.accept()
            print("Got connection from", addr)
            #Each new connection gets a new thread
            self.x = threading.Thread(target=self.start_socket, args=(conn, addr)).start()
            print(f'[Active Connections]: {threading.activeCount()}')
    
    def start_socket(self, conn, addr):
        while 1:
            try:
                if self.msg != None:
                    type_msg = self.msg.get_type()
                    if type_msg == 'GPS_INPUT':
                        self.send_gps(self.msg, conn)
                    elif type_msg == 'ATTITUDE':
                        self.send_attitude(self.msg, conn)
                    elif type_msg == 'BAD_DATA':
                        self.send_bad_data(self.msg, conn)
                time.sleep(self.threadlock_speed)
            except:
                conn.close()
                print(f"[INFO]: Closed the connection from {addr}")
                self.x.join()

    def recv_mavlink_msg(self):
        #Runs on a seperate thread and continuously gets the new MAVmsgs.
        while 1:
            self.msg = self.gc.recv_match()
            time.sleep(self.threadlock_speed)

    def encode_and_send(self, message, conn):
        message = message.encode()
        conn.sendall(message)

    def send_gps(self,msg, conn):
        message = f"[GPS, {float(msg.lat)}" + "," + f"{float(msg.lon)}" + "," + f"{float(msg.alt)}]"
        self.encode_and_send(message, conn)
    
    def convert_to_0_360(self,angle):
        if angle < 0:
            angle += 360
        return angle
    
    def send_attitude(self, msg, conn):
        msg.yaw = self.convert_to_0_360(msg.yaw)
        print(msg.yaw)
        message = f"[ATTITUDE, {msg.roll}" + "," + f"{msg.pitch}" + "," + f"{msg.yaw}]"
        self.encode_and_send(message, conn)

    def send_bad_data(self, msg, conn):
        message = f"[BAD_DATA]"
        self.encode_and_send(message, conn)

if __name__ == "__main__":
    comms = CommsGCS(HOST= "192.168.1.186", PORT=8080, device="/dev/ttyUSB0", baud=57600)