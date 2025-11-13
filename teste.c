#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265
#define ALTURA 89
#define LARGURA 89


uint8_t img[89][89]; //< Matriz que recebe a imagem de entrada
//double mag[89][89]; //< Matriz para armazenar a magnitude do gradiente
//double dir[89][89]; //< Matriz para armazenar a direção do gradiente
// uint8_t output[89][89]; //< Matriz que receberá a imagem de saída

uint64_t largura = LARGURA, altura = ALTURA; // atribuindo o valor das constantes às variáveis largura e altura

/**
 * @brief   Lê uma imagem PGM formato ASCII (P2) 89x89 e armazena em 'img'
 *
 * @param   filename    caminho do arquivo PGM
 *
 */
void ler_pgm(const char* filename) {
    FILE* f = fopen(filename, "r");
    if (!f) {
        printf("Erro ao abrir arquivo\n");
        return;
    }
    char tipo[3];
    int maxval;
    fscanf(f, "%2s", tipo);
    if (tipo[0] != 'P' || tipo[1] != '2') {
        printf("Arquivo não é PGM tipo P2 (ASCII)\n");
        fclose(f);
        return;
    }
    int c;
    do {
        c = fgetc(f);
        if (c == '#') while (fgetc(f) != '\n');
    } while (c == '#');
    ungetc(c, f);
    fscanf(f, "%d %d", &largura, &altura);
    fscanf(f, "%d", &maxval);

    if (largura != 89 || altura != 89) {
        printf("Dimensões da imagem não são 89x89!\n");
        fclose(f);
        return;
    }

    for (int i = 0; i < 89; i++) {
        for (int j = 0; j < 89; j++) {
            fscanf(f, "%d", &img[i][j]);
        }
    }
    fclose(f);
}

/**
 * @brief   Salva uma matriz 89x89 no formato PGM P2
 *
 * @param   nome_arquivo    nome do arquivo de saída
 * @param   matriz    matriz da imagem a ser escrita
 */
void salvar_pgm(const char* nome_arquivo, uint8_t matriz[89][89]) {
    FILE* f = fopen(nome_arquivo, "w");
    if (!f) {
        perror("Erro ao abrir arquivo para escrita");
        return;
    }
    fprintf(f, "P2\n89 89\n255\n");
    for (int i = 0; i < 89; i++) {
        for (int j = 0; j < 89; j++) {
            fprintf(f, "%d ", matriz[i][j]);
        }
        fprintf(f, "\n");
    }
    printf("imagem reescrita\n");
    fclose(f);
}

// -------------------------------------------------
// copia matriz (mesmo tipo e dimensões)
// -------------------------------------------------
void imgcopy_uint8(const uint8_t src[ALTURA][LARGURA], uint8_t dst[ALTURA][LARGURA]) {
    for (int i = 0; i < ALTURA; ++i)
        for (int j = 0; j < LARGURA; ++j)
            dst[i][j] = src[i][j];
}

// -------------------------------------------------
// filtro gaussiano (usa buffer temp com mesmas dims)
// -------------------------------------------------
void filtro_gaussiano(uint8_t img[ALTURA][LARGURA]) {
    static const float kernel[5][5] = {
            {2.0f/159.0f, 4.0f/159.0f, 5.0f/159.0f, 4.0f/159.0f, 2.0f/159.0f},
            {4.0f/159.0f, 9.0f/159.0f,12.0f/159.0f, 9.0f/159.0f, 4.0f/159.0f},
            {5.0f/159.0f,12.0f/159.0f,15.0f/159.0f,12.0f/159.0f, 5.0f/159.0f},
            {4.0f/159.0f, 9.0f/159.0f,12.0f/159.0f, 9.0f/159.0f, 4.0f/159.0f},
            {2.0f/159.0f, 4.0f/159.0f, 5.0f/159.0f, 4.0f/159.0f, 2.0f/159.0f}
    };

    uint8_t temp[ALTURA][LARGURA];

    for (int i = 0; i < ALTURA; ++i) {
        for (int j = 0; j < LARGURA; ++j) {
            float sum = 0.0f;
            for (int m = -2; m <= 2; ++m) {
                for (int n = -2; n <= 2; ++n) {
                    int y = i + m;
                    int x = j + n;
                    if (y < 0) y = 0;
                    if (x < 0) x = 0;
                    if (y >= ALTURA) y = ALTURA - 1;
                    if (x >= LARGURA) x = LARGURA - 1;
                    sum += img[y][x] * kernel[m + 2][n + 2];
                }
            }
            if (sum < 0.0f) sum = 0.0f;
            if (sum > 255.0f) sum = 255.0f;
            temp[i][j] = (uint8_t)(sum + 0.5f);
        }
    }

    imgcopy_uint8(temp, img);
}

// -------------------------------------------------
// convolution: image (uint8) with 3x3 kernel -> int output
// use int output to hold sums safely
// -------------------------------------------------
void convolution_int(const uint8_t image[ALTURA][LARGURA], const int kernel[3][3], int output[ALTURA][LARGURA]) {
    int pad = 1;
    for (int row = 0; row < ALTURA; ++row) {
        for (int col = 0; col < LARGURA; ++col) {
            int sum = 0;
            for (int i = -pad; i <= pad; ++i) {
                for (int j = -pad; j <= pad; ++j) {
                    int y = row + i;
                    int x = col + j;
                    if (y < 0 || y >= ALTURA || x < 0 || x >= LARGURA) continue;
                    sum += image[y][x] * kernel[i + pad][j + pad];
                }
            }
            output[row][col] = sum;
        }
    }
}

// -------------------------------------------------
// non-max suppression: grad_mag and grad_dir (float/double) -> uint8_t output
// -------------------------------------------------
void non_max_suppression(const float grad_mag[ALTURA][LARGURA], const float grad_dir[ALTURA][LARGURA],
                         uint8_t output[ALTURA][LARGURA]) {
    // inicializa bordas com zero
    for (int r = 0; r < ALTURA; ++r)
        for (int c = 0; c < LARGURA; ++c)
            output[r][c] = 0;

    for (int row = 1; row < ALTURA - 1; ++row) {
        for (int col = 1; col < LARGURA - 1; ++col) {
            float direction = grad_dir[row][col];
            float mag = grad_mag[row][col];

            // quantize direction to one of 4 sectors (0,45,90,135)
            float q = 0.0f, r = 0.0f;
            // normalize angle to [-180,180)
            while (direction <= -180.0f) direction += 360.0f;
            while (direction > 180.0f) direction -= 360.0f;

            if ((direction >= -22.5f && direction <= 22.5f) || (direction <= -157.5f || direction > 157.5f)) {
                q = grad_mag[row][col + 1];
                r = grad_mag[row][col - 1];
            } else if ((direction > 22.5f && direction <= 67.5f) || (direction <= -112.5f && direction > -157.5f)) {
                q = grad_mag[row + 1][col - 1];
                r = grad_mag[row - 1][col + 1];
            } else if ((direction > 67.5f && direction <= 112.5f) || (direction <= -67.5f && direction > -112.5f)) {
                q = grad_mag[row + 1][col];
                r = grad_mag[row - 1][col];
            } else {
                q = grad_mag[row - 1][col - 1];
                r = grad_mag[row + 1][col + 1];
            }

            if (mag >= q && mag >= r)
                output[row][col] = (uint8_t)(mag > 255.0f ? 255 : (int)(mag + 0.5f));
            else
                output[row][col] = 0;
        }
    }
}

// -------------------------------------------------
// Sobel: uses convolution_int, computes grad_mag and grad_dir (float),
// then calls non_max_suppression to produce uint8_t edge image (out_img)
// -------------------------------------------------
void sobel_edge_detection(const uint8_t image_in[ALTURA][LARGURA], uint8_t out_img[ALTURA][LARGURA]) {
    const int gx_kernel[3][3] = { {-1,0,1}, {-2,0,2}, {-1,0,1} };
    const int gy_kernel[3][3] = { {1,2,1}, {0,0,0}, {-1,-2,-1} };

    int grad_x[ALTURA][LARGURA];
    int grad_y[ALTURA][LARGURA];
    float grad_mag[ALTURA][LARGURA];
    float grad_dir[ALTURA][LARGURA];

    convolution_int(image_in, (int (*)[3])gx_kernel, grad_x);
    convolution_int(image_in, (int (*)[3])gy_kernel, grad_y);

    float max_val = 0.0f;
    for (int i = 0; i < ALTURA; ++i) {
        for (int j = 0; j < LARGURA; ++j) {
            float gx = (float)grad_x[i][j];
            float gy = (float)grad_y[i][j];
            grad_mag[i][j] = sqrtf(gx*gx + gy*gy);
            grad_dir[i][j] = atan2f(gy, gx) * (180.0f / (float)PI); // degrees
            if (grad_mag[i][j] > max_val) max_val = grad_mag[i][j];
        }
    }

    if (max_val <= 0.0f) max_val = 1.0f; // evita divisão por zero

    // normaliza para 0..255
    for (int i = 0; i < ALTURA; ++i)
        for (int j = 0; j < LARGURA; ++j)
            grad_mag[i][j] = (grad_mag[i][j] / max_val) * 255.0f;

    // supressão não-máxima -> out_img
    non_max_suppression(grad_mag, grad_dir, out_img);
}

// -------------------------------------------------
// threshold in-place (uint8_t image -> values 0, weak, 255)
// -------------------------------------------------
void threshold_inplace(uint8_t img[ALTURA][LARGURA], int low, int high, uint8_t weak) {
    for (int i = 0; i < ALTURA; ++i) {
        for (int j = 0; j < LARGURA; ++j) {
            uint8_t v = img[i][j];
            if (v >= high) img[i][j] = 255;
            else if (v >= low) img[i][j] = weak;
            else img[i][j] = 0;
        }
    }
}

// -------------------------------------------------
// has_strong_neighbor adjusted for uint8_t
// -------------------------------------------------
static int has_strong_neighbor_uint8(const uint8_t img[ALTURA][LARGURA], int l, int c) {
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            int y = l + dy;
            int x = c + dx;
            if (y < 0 || y >= ALTURA || x < 0 || x >= LARGURA) continue;
            if (img[y][x] == 255) return 1;
        }
    }
    return 0;
}

// -------------------------------------------------
// hysteresis in-place (works on uint8_t image using weak/strong)
// -------------------------------------------------
void hysteresis_inplace(uint8_t img[ALTURA][LARGURA], uint8_t weak) {
    // cria cópia para leituras consistentes
    uint8_t copy_img[ALTURA][LARGURA];
    imgcopy_uint8(img, copy_img);

    for (int i = 1; i < ALTURA - 1; ++i) {
        for (int j = 1; j < LARGURA - 1; ++j) {
            if (copy_img[i][j] == weak) {
                if (has_strong_neighbor_uint8(copy_img, i, j))
                    img[i][j] = 255;
                else
                    img[i][j] = 0;
            }
        }
    }
}
/**
 * @brief   função main
 *
 * executa o pipeline do Canny:
 *
 * - leitura PGM;
 *
 * - filtro gaussiano;
 *
 * - sobel;
 *
 * - supressão não-máxima;
 *
 * - limiarização + histerese;
 *
 * @param   argc    número de argumentos
 * @param   argv [1]    arquivo entrada .pgm; [2]   saída .pgm
 */
int main(int argc, char *argv[]) {
    if(argc < 3) {
        printf("como usar: %s <input.pgm> <output.pgm>\n", argv[0]);
        return 1;
    }

    printf("lendo imagem...\n");
    ler_pgm(argv[1]);

    printf("filtro gaussiano...\n");
    filtro_gaussiano(img);
    salvar_pgm("gauss.pgm", img);

    printf("calculando gradientes...\n");
    sobel_edge_detection(img, img);
    salvar_pgm("supr.pgm", img);

    printf("supr nao-maxima...\n");
    //non_max_suppression(img);
    //salvar_pgm("supr.pgm", img);

    printf("fazendo a limiarização...\n");
    int weak = 75;
    threshold_inplace(img, 5, 20, 75);
    salvar_pgm("thresh.pgm", img);

    printf("finalizando com histerese...\n");
    hysteresis_inplace(img, weak);

    printf("salvando img...\n");
    salvar_pgm("output.pgm", img);
}