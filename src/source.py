import numpy as np
import serial
from time import sleep

PORT = 'COM7'
BAUD = 115200
IMG_HEIGHT = 89
IMG_WIDTH = 89
BANDA_SOBEL = 3

serial = serial.Serial('COM7',115200,timeout=2)

def send_band(ser, band):
    serial.write(band.astype(np.uint8).tobytes())
    
def receive_line(ser, length=IMG_WIDTH):
    line = serial.read(length)
    assert len(line) == length, f"Recepção falhou: {len(line)} bytes lidos"
    
    return np.frombuffer(line, dtype=np.uint8)