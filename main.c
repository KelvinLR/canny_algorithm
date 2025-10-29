/** ALGORITMO CANNY P/ DETECÇÃO DE BORDAS
 * 
 * PASSO-A-PASSO
 * \_ 1) aplicação de filtro gaussiano 2D
 * \_ 2) encontrar gradiente utilizando Sobel
 * \_ 3) supressão não-máxima
 * \_ 4) limiarização com histerese
 * 
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265
#define KERNEL_SIZE 5
#define K 2
#define ALTURA 89
#define LARGURA 89

uint64_t img[89][89];
double mag[89][89];
double dir[89][89];
uint64_t output[89][89];
uint64_t largura = LARGURA, altura = ALTURA;

void ler_pgm(const char* filename) {
    FILE* f = fopen(filename, "r");
    if (!f) {
        printf("Erro ao abrir arquivo\n");
        return;
    }
    char tipo[3];
    //int largura, altura, maxval;
    int maxval;
    fscanf(f, "%2s", tipo);
    if (tipo[0] != 'P' || tipo[1] != '2') {
        printf("Arquivo não é PGM tipo P2 (ASCII)\n");
        fclose(f);
        return;
    }
    // Pula comentários, se houver
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

void salvar_pgm(const char* nome_arquivo, int matriz[89][89]) {
    FILE* f = fopen(nome_arquivo, "w");
    if (!f) {
        perror("Erro ao abrir arquivo para escrita");
        return;
    }
    // Cabeçalho PGM tipo P2
    fprintf(f, "P2\n89 89\n255\n");
    for (int i = 0; i < 89; i++) {
        for (int j = 0; j < 89; j++) {
            // Certifique-se que matriz[i][j] está entre 0 e 255!
            fprintf(f, "%d ", matriz[i][j]);
        }
        fprintf(f, "\n");
    }
    printf("imagem reescrita\n");
    fclose(f);
}

void imgcopy(const int input[ALTURA][LARGURA], int output[ALTURA][LARGURA]) {
    for(int i = 0; i < ALTURA; ++i)
        for(int j = 0; j < LARGURA; ++j)
            output[i][j] = input[i][j];
}

void filtro_gaussiano() {
    int temp[89][89]; // buffer temporário
    float kernel[5][5] = {
        {2.0f/159.0f, 4.0f/159.0f, 5.0f/159.0f, 4.0f/159.0f, 2.0f/159.0f},
        {4.0f/159.0f, 9.0f/159.0f, 12.0f/159.0f, 9.0f/159.0f, 4.0f/159.0f},
        {5.0f/159.0f, 12.0f/159.0f, 15.0f/159.0f, 12.0f/159.0f, 5.0f/159.0f},
        {4.0f/159.0f, 9.0f/159.0f, 12.0f/159.0f, 9.0f/159.0f, 4.0f/159.0f},
        {2.0f/159.0f, 4.0f/159.0f, 5.0f/159.0f, 4.0f/159.0f, 2.0f/159.0f}
    };

    int k = 2; // kernel_size / 2
    int i, j, m, n;
    float sum;
    int row_idx, col_idx;

    // Percorre cada pixel da imagem de saída
    for(i = 0; i < altura; i++) {
        for(j = 0; j < largura; j++) {
            sum = 0.0f;

            // Aplica o kernel na região da imagem considerando o padding com "modo-edge"
            for(m = -k; m <= k; m++) {
                for(n = -k; n <= k; n++) {
                    row_idx = i + m;
                    if(row_idx < 0) row_idx = 0;
                    else if(row_idx >= altura) row_idx = altura - 1;
                    col_idx = j + n;
                    if(col_idx < 0) col_idx = 0;
                    else if(col_idx >= largura) col_idx = largura - 1;
                    sum += img[row_idx][col_idx] * kernel[m + k][n + k];
                }
            }

            // Clamp para faixa [0, 255], SEM arredondamento ― igual Python int()
            if(sum < 0.0f) sum = 0.0f;
            if(sum > 255.0f) sum = 255.0f;
            temp[i][j] = (int)sum; // truncamento, igual Python

        }
    }

    imgcopy(temp, img);
}

void convolution(int image[ALTURA][LARGURA], int kernel[3][3], int output[ALTURA][LARGURA]) {
    int pad = 1;
    for(int row=0; row<ALTURA; row++) {
        for(int col=0; col<LARGURA; col++) {
            int sum = 0;
            for(int i=-pad; i<=pad; i++) {
                for(int j=-pad; j<=pad; j++) {
                    int y = row + i, x = col + j;
                    if(y < 0 || y >= ALTURA || x < 0 || x >= LARGURA)
                        continue;
                    sum += image[y][x] * kernel[i+pad][j+pad];
                }
            }
            output[row][col] = sum;
        }
    }
}

void sobel_edge_detection(int image[ALTURA][LARGURA], double grad_mag[ALTURA][LARGURA], double grad_dir[ALTURA][LARGURA]) {
    int gx_kernel[3][3] = {
        {-1,0,1},
        {-2,0,2},
        {-1,0,1}
    };
    int gy_kernel[3][3] = {
        {1,2,1},
        {0,0,0},
        {-1,-2,-1},
    };

    int grad_x[ALTURA][LARGURA], grad_y[ALTURA][LARGURA];

    convolution(image, gx_kernel, grad_x);
    convolution(image, gy_kernel, grad_y);

    double max_val = 0.0f;
    for(int i=0;i<ALTURA;i++)for(int j=0;j<LARGURA;j++) {
        grad_mag[i][j] = sqrt(grad_x[i][j]*grad_x[i][j] + grad_y[i][j]*grad_y[i][j]);
        if(grad_mag[i][j]>max_val)
            max_val = grad_mag[i][j];
    }
    // Normaliza para 0..255
    for(int i=0;i<ALTURA;i++)for(int j=0;j<LARGURA;j++)
        grad_mag[i][j] = grad_mag[i][j] * 255.0f / max_val;

    for(int i=0;i<ALTURA;i++)for(int j=0;j<LARGURA;j++)
        grad_dir[i][j] = atan2f(grad_y[i][j], grad_x[i][j]) * (180.0f /PI) + 180;

    FILE *f = fopen("grad_dir.txt", "w");
    if (!f) {
        perror("Erro ao abrir arquivo");
        exit(1);
    }
    for (int i = 0; i < 89; i++) {
        for (int j = 0; j < 89; j++) {
            fprintf(f, "%.15f\n", grad_dir[i][j]); // float ou int, conforme o tipo
        }
    }
    fclose(f);

    printf("cheguei no sobel\n");
}

void non_max_suppression(double grad_mag[ALTURA][LARGURA],
                         double grad_dir[ALTURA][LARGURA],
                         int output[ALTURA][LARGURA]) {
    for (int row = 0; row < ALTURA; row++) {
        for (int col = 0; col < LARGURA; col++) {
            if(row == 0 || col == 0 || row == ALTURA-1 || col == LARGURA-1) {
                output[row][col] = 0;
                continue;
            }
            double direction = grad_dir[row][col];
            /*if (direction < 0) {
                direction += 180;
            }*/
            double mag = grad_mag[row][col];
            int by, bx, ay, ax;
            double before_pixel = 0, after_pixel = 0;

            if ((0 <= direction && direction < 22.5) ||
                (337.5 <= direction && direction <= 360)) {
                ay = by = row;
                bx = col - 1, ax = col + 1;
            } else if ((22.5 <= direction && direction < 67.5) ||
                (202.5 <= direction && direction < 247.5)) {
                by = row + 1, bx = col - 1;
                ay = row - 1, ax = col + 1;
            } else if ((67.5 <= direction && direction < 112.5) ||
                (247.5 <= direction && direction < 292.5)) {
                by = row - 1, ay = row + 1;
                bx = ax = col;
            } else {
                by = row - 1, bx = col - 1;
                ay = row + 1, ax = col + 1;
            }

            before_pixel = grad_mag[by][bx];
            after_pixel = grad_mag[ay][ax];

            /*if (by >= 0 && bx >= 0 && by < ALTURA && bx < LARGURA)
                before_pixel = grad_mag[by][bx];
            if (ay >= 0 && ax >= 0 && ay < ALTURA && ax < LARGURA)
                after_pixel = grad_mag[ay][ax];*/
            if (mag >= before_pixel && mag >= after_pixel)
                output[row][col] = (int)mag;
            else
                output[row][col] = 0;

        }
    }
    FILE *f = fopen("supr.txt", "w");
    if (!f) {
        perror("Erro ao abrir arquivo");
        exit(1);
    }
    for (int i = 0; i < 89; i++) {
        for (int j = 0; j < 89; j++) {
            fprintf(f, "%d.0\n", output[i][j]); // float ou int, conforme o tipo
        }
    }
    fclose(f);
    for (int i = 0; i < 89; i++) {
        for (int j = 0; j < 89; j++) {
            printf("%d.0", output[i][j]); // float ou int, conforme o tipo
        }
        printf("\n");
    }
    printf("cheguei aqui na supressao\n");
}

void threshold(int img[ALTURA][LARGURA], int low, int high, int weak, int output[ALTURA][LARGURA]) {
    for(int i=0;i<ALTURA;i++)
        for(int j=0;j<LARGURA;j++) {
            if(img[i][j] >= high)
                output[i][j] = 255;
            else if(img[i][j] >= low)
                output[i][j] = weak;
            else
                output[i][j] = 0;
        }
    printf("cheguei aqui no threshold\n");
}

static int has_strong_neighbor(const int img[ALTURA][LARGURA], int l, int c) {
    int strong = 255;
    const int movs[] = {-1, 0, 1};
    for(int i = 0; i < 3; ++i) {
        int nl = l + movs[i];
        if (nl < 0 || nl >= ALTURA) continue;
        for (int j = 0; j < 3; ++j) {
            int nc = c + movs[j];
            if (nc < 0 || nc >= LARGURA) continue;
            if (img[nl][nc] == strong) return 1;
        }
    }
    return 0;
}

void hysteresis(int img[ALTURA][LARGURA], int weak) {
    // 4 passes: top-down, bottom-up, left-right, right-left
    int top_bottom[ALTURA][LARGURA], bottom_up[ALTURA][LARGURA];
    int left_right[ALTURA][LARGURA], right_left[ALTURA][LARGURA];
    imgcopy(img, top_bottom);
    imgcopy(img, bottom_up);
    imgcopy(img, left_right);
    imgcopy(img, right_left);

    int strong = 255;
    // Pass 1: top_to_bottom
    int i = 0, j = 0;
    for (i = 1; i < ALTURA; ++i)
        for (j = 1; j < LARGURA; ++j)
            if (top_bottom[i][j] == weak) {
                if (has_strong_neighbor(top_bottom, i, j)) {
                    top_bottom[i][j] = strong;
                } else {
                    top_bottom[i][j] = 0;
                }
            };
    for (i = ALTURA - 2; i >= 1; --i)
        for (j = LARGURA - 2; j >= 1; --j)
            if (bottom_up[i][j] == weak) {
                if (has_strong_neighbor(bottom_up, i, j)) {
                    bottom_up[i][j] = strong;
                } else {
                    bottom_up[i][j] = 0;
                }
            };
    for (i = 1; i < ALTURA - 1; ++i)
        for (j = LARGURA - 2; j >= 1; --j)
            if (right_left[i][j] == weak) {
                if (has_strong_neighbor(right_left, i, j)) {
                    right_left[i][j] = strong;
                } else {
                    right_left[i][j] = 0;
                }
            };
    for (i = ALTURA - 2; i >= 1; --i)
        for (j = 1; j < LARGURA - 1; ++j)
            if (left_right[i][j] == weak) {
                if (has_strong_neighbor(left_right, i, j)) {
                    left_right[i][j] = strong;
                } else {
                    left_right[i][j] = 0;
                }
            };

    for (i = 0; i < ALTURA; ++i) {
        for (j = 0; j < LARGURA; ++j) {
            int soma = top_bottom[i][j] + bottom_up[i][j] + left_right[i][j] +
                       right_left[i][j];
            img[i][j] = (soma > 255) ? 255 : soma;
        }
    }

    printf("cheguei aqui na histerese");
}

float compare_images(int img1[89][89], int img2[89][89]) {
    const int altura = 89;
    const int largura = 89;
    float total_diff = 0.0f;

    // Para PGM grayscale - apenas 1 canal
    for (int x = 0; x < largura; x++) {
        for (int y = 0; y < altura; y++) {
            int diff = img1[y][x] - img2[y][x];
            // Valor absoluto
            if (diff < 0) {
                total_diff += (float)(-diff) / 255.0f;
            } else {
                total_diff += (float)diff / 255.0f;
            }
        }
    }

    // Percentual de diferença
    float percentual = 100.0f * total_diff / (float)(largura * altura);

    printf("percentual de diferença entre as duas imagens: %.4f", percentual);
    return percentual;
}

int main() {

    printf("Lendo imagem...\n");
    ler_pgm("./agathar.pgm");

    printf("Aplicando filtro gaussiano...\n");
    filtro_gaussiano();
    salvar_pgm("gauss.pgm", img);

    printf("Calculando gradientes...\n");
    //calc_gradiente();
    sobel_edge_detection(img, mag, dir);

    printf("Supressão não-máxima...\n");
    //supr_nao_max();
    non_max_suppression(mag, dir, img);
    salvar_pgm("supr.pgm", img);

    printf("Realizando limiarização com histereses 30 e 75...\n");
    //limiarizacao_histerese(30, 75);
    int weak = 75;
    threshold(img, 30, 75, weak, output);
    salvar_pgm("thresh.pgm", output);

    hysteresis(output, weak);

    printf("Sobrescrevendo e salvando imagem...\n");
    salvar_pgm("output.pgm", output);
}
