import struct
import numpy as np
import serial
import time
import matplotlib.pyplot as plt

ser = serial.Serial("COM5", 115200, timeout=1)



def read_pgm(path):
    """
    Lê um arquivo PGM (formato P2 - ASCII) e retorna uma matriz numpy uint8.
    Ignora comentários e linhas em branco.
    """
    with open(path, 'r') as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith('#')]

    if not lines or lines[0] != 'P2':
        raise ValueError("Formato inválido. O arquivo deve ser P2 (ASCII).")

    # Lê largura e altura
    try:
        width, height = map(int, lines[1].split())
        maxval = int(lines[2])
        pixels = np.array([int(x) for x in " ".join(lines[3:]).split()], dtype=np.uint8)
    except Exception as e:
        raise ValueError(f"Erro ao ler PGM: {e}")

    if pixels.size != width * height:
        raise ValueError(f"Tamanho inconsistente: esperava {width*height} pixels, recebeu {pixels.size}")

    return pixels.reshape((height, width))

def pad_tile(tile, size):
    h, w = tile.shape
    padded = np.zeros((size, size), dtype=np.uint8)
    padded[:h, :w] = tile
    return padded, h, w

img = read_pgm("cereja.pgm") 
H, W = img.shape
TILE = 45
reconstructed = np.zeros_like(img)

for y in range(0, H, TILE):
    for x in range(0, W, TILE):
        sub = img[y:min(y+TILE, H), x:min(x+TILE, W)]
        padded, h, w = pad_tile(sub, TILE)

        # Cabeçalho + byte de sincronismo
        header = struct.pack("<HHHH", h, w, x, y)
        ser.write(b'S')  # byte de start
        ser.write(header)
        ser.write(padded.tobytes())

        # Recebe eco
        sync = ser.read(1)
        if sync != b'S':
            print("❌ Erro de sincronismo, ignorando tile")
            ser.reset_input_buffer()
            continue

        rx_header = ser.read(8)
        rx_data = ser.read(TILE*TILE)

        if len(rx_header) != 8 or len(rx_data) != TILE*TILE:
            print("❌ Tamanho incorreto, pulando tile")
            continue

        rh, rw, rx, ry = struct.unpack("<HHHH", rx_header)
        tile = np.frombuffer(rx_data, dtype=np.uint8).reshape((TILE, TILE))

        # --- Limita ao tamanho da imagem ---
        y_end = min(ry + rh, H)
        x_end = min(rx + rw, W)
        h_eff = y_end - ry
        w_eff = x_end - rx

        # Protege contra indices inválidos
        if ry >= H or rx >= W or h_eff <= 0 or w_eff <= 0:
            print(f"⚠️ Tile fora da imagem: ({rx},{ry}) ignorado")
            continue

        reconstructed[ry:y_end, rx:x_end] = tile[:h_eff, :w_eff]
        end_flag = ser.read(1)
        print(f"✅ Tile ({x},{y}) ecoado ({rh}x{rw}) fim={end_flag}")
        time.sleep(5)

ser.close()

plt.subplot(1,2,1)
plt.title("Original")
plt.imshow(img, cmap="gray")
plt.subplot(1,2,2)
plt.title("Ecoada")
plt.imshow(reconstructed, cmap="gray")
plt.show()
