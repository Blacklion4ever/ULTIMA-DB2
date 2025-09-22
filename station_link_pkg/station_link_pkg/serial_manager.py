# Node serial_manager.py - à implémenter
import serial
from common_link_lib.frame_manager import FrameManager

class SerialManager:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.01)
        self.fm = FrameManager()

    def send_frame(self, header, subheader, payload_bytes):
        frame = self.fm.build_frame(header, subheader, payload_bytes)
        self.ser.write(frame)

    def read_frames(self):
        data = self.ser.read(64)
        return self.fm.feed(data)
