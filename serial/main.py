import struct
import numpy as np
import serial
import time
import matplotlib.pyplot as plt

ser = serial.Serial("COM9", 115200, timeout=5)

def read_pgm(path):
    with open(path, 'r') as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith('#')]
    if not lines or lines[0] != 'P2':
        raise ValueError("Formato inválido. O arquivo deve ser P2 (ASCII).")
    width, height = map(int, lines[1].split())
    maxval = int(lines[2])
    pixels = np.array([int(x) for x in " ".join(lines[3:]).split()], dtype=np.uint8)
    if pixels.size != width * height:
        raise ValueError(f"Tamanho inconsistente: esperava {width*height} pixels, recebeu {pixels.size}")
    return pixels.reshape((height, width))

def save_pgm(path, image, maxval=255):
    height, width = image.shape
    with open(path, 'w') as f:
        f.write("P2\n")
        f.write(f"{width} {height}\n")
        f.write(f"{maxval}\n")

        count = 0
        for val in image.flatten():
            f.write(f"{int(val)} ")
            count += 1
            if count % 20 == 0:
                f.write("\n")  

def make_padded_from_image(img, x, y, TILE_TOTAL, TILE_UTIL, BORDER):
    """
    Constrói padded de tamanho TILE_TOTAL x TILE_TOTAL a partir da imagem,
    fazendo clamp nas bordas (replicação).
    x,y = coordenada (coluna, linha) do canto superior esquerdo da tile util.
    """
    H, W = img.shape
    padded = np.zeros((TILE_TOTAL, TILE_TOTAL), dtype=np.uint8)
    # cada posição i,j do padded corresponde a img[ y - BORDER + i, x - BORDER + j ] com clamp
    for i in range(TILE_TOTAL):
        img_y = y - BORDER + i
        if img_y < 0:
            img_y = 0
        elif img_y >= H:
            img_y = H - 1
        for j in range(TILE_TOTAL):
            img_x = x - BORDER + j
            if img_x < 0:
                img_x = 0
            elif img_x >= W:
                img_x = W - 1
            padded[i, j] = img[img_y, img_x]
    return padded

img = read_pgm("canny_algorithm/serial/melbourne.pgm")
H, W = img.shape

TILE_UTIL = 16
BORDER = 4
TILE_TOTAL = TILE_UTIL + 2 * BORDER

reconstructed = np.zeros_like(img)

for y in range(0, H, TILE_UTIL):      
    for x in range(0, W, TILE_UTIL):   
        padded = make_padded_from_image(img, x, y, TILE_TOTAL, TILE_UTIL, BORDER)

        header = struct.pack("<HHHH", TILE_TOTAL, TILE_TOTAL, x, y)
        ser.write(b'S')
        ser.write(header)
        ser.write(padded.tobytes())

        sync = ser.read(1)
        if sync != b'S':
            print("❌ Erro de sincronismo, ignorando tile")
            ser.reset_input_buffer()
            continue

        rx_header = ser.read(8)
        rx_data = ser.read(TILE_TOTAL * TILE_TOTAL)

        if len(rx_header) != 8 or len(rx_data) != TILE_TOTAL * TILE_TOTAL:
            print("❌ Tamanho incorreto, pulando tile")
            continue

        rh, rw, rx, ry = struct.unpack("<HHHH", rx_header)
        tile = np.frombuffer(rx_data, dtype=np.uint8).reshape((TILE_TOTAL, TILE_TOTAL))

        # corta borda e coloca no lugar certo
        useful = tile[BORDER:BORDER+TILE_UTIL, BORDER:BORDER+TILE_UTIL]

        y_end = min(ry + TILE_UTIL, H)
        x_end = min(rx + TILE_UTIL, W)
        reconstructed[ry:y_end, rx:x_end] = useful[:y_end-ry, :x_end-rx]

        end_flag = ser.read(1)
        print(f"✅ Tile ({x},{y}) ecoado ({rh}x{rw}) fim={end_flag}")
        time.sleep(0.02)
        
ser.close()

save_pgm("canny_algorithm/serial/reconstructed.pgm", reconstructed)

plt.subplot(1,2,1)
plt.title("Original")
plt.imshow(img, cmap="gray")
plt.subplot(1,2,2)
plt.title("Ecoada")
plt.imshow(reconstructed, cmap="gray")
plt.show()
