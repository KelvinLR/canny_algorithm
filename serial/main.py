"""
Arquivo: main.py
Descrição:
    Implementa o envio de tiles de imagem para um sistema embarcado via porta serial,
    incluindo empacotamento, transmissão, recepção e reconstrução da imagem completa.
    O script realiza:
        - Leitura de imagem PGM (formato P2)
        - Montagem de tiles com bordas replicadas (padding)
        - Envio dos tiles para um dispositivo STM32
        - Recebimento dos tiles processados
        - Reconstrução da imagem final
        - Salvamento e visualização dos resultados

Objetivo:
    Fornecer uma pipeline completa para realizar processamento distribuído de imagens
    em janelas (tiling) com contexto, permitindo que cada bloco seja processado
    independentemente no hardware embarcado.

Dependências:
    - numpy
    - serial (pyserial)
    - struct
    - matplotlib
    - time

Funções principais:
    read_pgm(path):
        Lê uma imagem no formato PGM ASCII (P2) e retorna um array numpy.

    save_pgm(path, image, maxval=255):
        Salva um array numpy no formato PGM ASCII.

    make_padded_from_image(img, x, y, TILE_TOTAL, TILE_UTIL, BORDER):
        Gera um tile com bordas replicadas ao redor da região útil da imagem.

Notas:
    O protocolo serial envolve:
        - byte 'S' de sincronização
        - cabeçalho com dimensões e posição do tile
        - envio/recebimento dos dados brutos do tile
    A reconstrução usa apenas a região útil do tile retornado.

Autores:
    Kelvin de Lima Rodrigues
    Marcelo Antônio Dantas Filho

Data:
    14/11/2025
"""
import struct
import numpy as np
import serial
import time
import matplotlib.pyplot as plt

ser = serial.Serial("COM7", 115200, timeout=3)

# funções de ler e salvar PGM
def read_pgm(path):
    """Lê uma imagem PGM no formato ASCII (P2) e retorna como um numpy array.

    Args:
        path (string): Caminho da imagem PGM

    Raises:
        ValueError: Formato inválido
        ValueError: Tamanho inconsistente

    Returns:
        pixels (ndarray): Imagem em numpy array
    """
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
    """Salva uma imagem numpy array no formato PGM ASCII (P2).

    Args:
        path (string): Caminho para salvar a imagem PGM
        image (ndarray): Imagem em numpy array
        maxval (int, optional): Valor máximo atribuído por pixel. Default 255.
    """
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

# vai fazer um "padded" da imagem do tamanho do tile total, mas colocando o tile util e bordas
def make_padded_from_image(img, x, y, TILE_TOTAL, TILE_UTIL, BORDER):
    """Gera um tile "padded" a partir da imagem original, aplicando bordas replicadas.

    A função cria um tile quadrado de tamanho TILE_TOTAL x TILE_TOTAL que contém:
    - A região útil (TILE_UTIL x TILE_UTIL) ao redor da posição (x, y) da imagem original.
    - Bordas (padding) replicadas para permitir processamento em janelas com contexto.

    Args:
        img (ndarray): Imagem completa de entrada (H x W).
        x (int): Coordenada X (coluna) do canto superior-esquerdo da região útil.
        y (int): Coordenada Y (linha) do canto superior-esquerdo da região útil.
        TILE_TOTAL (int): Tamanho total do tile enviado ao sistema embarcado
            (útil + 2*borda).
        TILE_UTIL (int): Tamanho apenas da área útil que será processada.
        BORDER (int): Quantidade de pixels de borda adicionados ao redor do tile.

    Returns:
        ndarray: Tile preenchido com valores da imagem usando replicação de bordas,
        com formato (TILE_TOTAL, TILE_TOTAL).
    """

    H, W = img.shape
    padded = np.zeros((TILE_TOTAL, TILE_TOTAL), dtype=np.uint8)

    # percorre todos os pixels do tile de destino padded
    # calcula quais coordenadas na img de entrada vao ser copiadas para cada ponto do tile
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

# lendo a imagem completa
img = read_pgm("cereja.pgm")
# obtendo largura e altura da imagem
H, W = img.shape

# essas variáveis auxiliam no processamento, pois estamos trabalhando com
# processamento em janelas, o tile total vai ser o tanto q o STM vai receber
# o tile util vai ser o tanto que vai ser processado e o border é necessário 
# para pegar o "contexto" q sao os elementos próximos p reduzir os impactos do tile processing
TILE_UTIL = 16
BORDER = 4
TILE_TOTAL = TILE_UTIL + 2 * BORDER

# aqui essa matriz ta inicializada c zeros e vai armazenar a img reconstruída
reconstructed = np.zeros_like(img)

# divide em blocos do tamanho do tile util para o embarcado processar
for y in range(0, H, TILE_UTIL):      
    for x in range(0, W, TILE_UTIL):   
        # monta um tile padded de tamanho TILE_TOTAL (tamanho útil + bordas BORDER p pegar contexto)
        padded = make_padded_from_image(img, x, y, TILE_TOTAL, TILE_UTIL, BORDER)

        # cria um cabeçalho c os dados do tile, incluindo dimensões e posição (x, y)
        # envia um byte de sincronismo 'S', o cabeçalho e o tile completo via serial
        header = struct.pack("<HHHH", TILE_TOTAL, TILE_TOTAL, x, y)
        ser.write(b'S')
        ser.write(header)
        ser.write(padded.tobytes())

        # recebe um byte de volta para garantir sincronia, caso contrário ignora o tile atual
        sync = ser.read(1)
        if sync != b'S':
            print("❌ Erro de sincronismo, ignorando tile")
            ser.reset_input_buffer()
            continue

        # recebe  cabeçalho e tile processada
        rx_header = ser.read(8)
        rx_data = ser.read(TILE_TOTAL * TILE_TOTAL)

        if len(rx_header) != 8 or len(rx_data) != TILE_TOTAL * TILE_TOTAL:
            print("❌ Tamanho incorreto, pulando tile")
            continue

        # desempacota as informações no cabeçalho e
        # converte os dados recebidos para matriz numpy
        rh, rw, rx, ry = struct.unpack("<HHHH", rx_header)
        tile = np.frombuffer(rx_data, dtype=np.uint8).reshape((TILE_TOTAL, TILE_TOTAL))

        # corta borda e coloca no lugar certo
        useful = tile[BORDER:BORDER+TILE_UTIL, BORDER:BORDER+TILE_UTIL]
        
        # grante que não sejam acessadas posições fora da imagem original
        y_end = min(ry + TILE_UTIL, H)
        x_end = min(rx + TILE_UTIL, W)

        # reconstrói a imagem
        reconstructed[ry:y_end, rx:x_end] = useful[:y_end-ry, :x_end-rx]

        # espera um byte q indica do fim do processamento do tile.
        end_flag = ser.read(1)
        print(f"✅ Tile ({x},{y}) ecoado ({rh}x{rw}) fim={end_flag}")
        # delay de 0.02s entre cada tile
        time.sleep(0.02)

# fecha a comunicação serial
ser.close()

save_pgm("cereja_reconstructed.pgm", reconstructed)

plt.subplot(1,2,1)
plt.title("Original")
plt.imshow(img, cmap="gray")
plt.subplot(1,2,2)
plt.title("Processada")
plt.imshow(reconstructed, cmap="gray")
plt.show()
