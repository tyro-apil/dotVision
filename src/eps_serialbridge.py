#! /usr/bin/env python3

# -------------------------SERIAL BRIDGE BETWEEN RASP PI AND STM32-------------------------------
# This node subscribes to
#   I) 'cmd_robot'_vel topic to recieve velocity commands
#   II) 'act_vel' from BT to recieve actuator control signals, and  write the data to stm discovery

# This node reads odometry value from bluepill and publish the data on
#   I) 'freewheel/odom' topic to ekf filter package through
#   II)'Ball_status' topic to BT

import math
import struct
import time
from math import cos, sin


from serial_lib.serial_comms import serial_comms
from serial_lib.parser import parser
from serial_lib.angler import angler

START_BYTE = 0b10100101
RECIEVE_SIZE = 1 + 24 + 24 + 1
TRANSMIT_SIZE = 1 + 15 + 1
serial_baudrate = 115200

serial_esp = "/dev/ttyUSB0"


class esp_bridge():
    def __init__(self):
        super().__init__("serial_bridge")
        self.write_serial_port = serial_comms(
            serial_esp,
            serial_baudrate,
            RECIEVE_SIZE,
            TRANSMIT_SIZE,
            "CSUM",
        )
        # self.read_serial_port = serial_comms(
        #     serial_port_stm,
        #     serial_baudrate,
        #     RECIEVE_SIZE,
        #     TRANSMIT_SIZE,
        #     "CRC",
        # )

        # self.timer2 = self.create_timer(0.015, self.Send_Data_CallBack)
        self.timer1 = self.create_timer(0.001, self.serial_read_callback)
        self.parser = parser()
        self.angler = angler()

        self.rx_data = [0.0] * 24
        self.x_offset = 0.0
        self.y_offset = 0.0

        self.cmd_vel_last_rx_time = time.time()
        self.act_vel_last_rx_time = time.time()

        self.last_transmit_time = time.time()
        self.last_published_time = time.time()

        self.get_logger().info("Serial bridge ready...")

    # Joystick read callback function
    def serial_write(self):
        now = time.time()
        if now - self.last_transmit_time < 0.01:
            return
        

        f = open("myfile.txt", "r")
        ch = f.read()
        braile_text = parser.text_to_braille(ch)
        angles = Angler.braille_to_angle(braile_text)

        DataToSend = [
            bytes(struct.pack("B", START_BYTE)),
            bytes(struct.pack("f", self.cmd_vel_msg.data[0])),
            bytes(struct.pack("f", self.cmd_vel_msg.data[1])),
            bytes(struct.pack("f", self.cmd_vel_msg.data[2])),
            bytes(struct.pack("B", self.act_vel_msg.data[0])),
            bytes(struct.pack("B", self.act_vel_msg.data[1])),
            bytes(struct.pack("B", self.act_vel_msg.data[2])),
        ]
        DataToSend = b"".join(DataToSend)
        data_hash = self.calculate_crc(DataToSend[1:])
        DataToSend = [DataToSend, bytes(struct.pack("B", data_hash))]
        DataToSend = b"".join(DataToSend)
        # print(f"{data_hash =}")
        # print(f"{self.cmd_vel_msg.data[2]}")
        self.last_transmit_time = time.time()

        self.write_serial_port.write_data(DataToSend)
        self.cmd_vel_rx_flag = False
        self.act_vel_rx_flag = False


    def serial_read(self):
        self.Send_Data_CallBack()
        _data = self.write_serial_port.read_data()
        if _data is not None:
            """data = [x, y, theta, vx, vy, omega, imu_data[6]]"""
            data = struct.unpack("ffffffffffff", _data[0:-1])
            self.rx_data = data
            # self.rx_data[1] = self.rx_data[1]

            self.process_odom(self.rx_data[0:9])
            self.process_imu(self.rx_data[6:])
            now = time.time()

            delay = now - self.last_published_time
            if delay >= 0.02:
                print(f"{delay = }")
            # print(f"{data =}")

            self.last_published_time = time.time()

        # else:
        #     print("data none")

  
    def calculate_checksum(self, data=[]):
        digest = int()
        for i in range(1, len(data)):
            digest += data[i]
        return int(digest)

def main(args=None):
    bridge = esp_bridge()
    
    while True:
        bridge.serial_write()
        bridge.serial_read()


if __name__ == "__main":
    main()
