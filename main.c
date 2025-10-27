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
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265
#define KERNEL_SIZE 5
#define K 2
#define ALTURA 89
#define LARGURA 89

int img[89][89];
float mag[89][89];
float dir[89][89];
int output[89][89];
int largura = LARGURA, altura = ALTURA;

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

            img[i][j] = temp[i][j];
        }
    }

    salvar_pgm("gauss.pgm", img);
}

void convolution(int image[ALTURA][LARGURA], int kernel[3][3], int output[ALTURA][LARGURA]) {
    int pad = 1;
    for(int row=0; row<ALTURA; row++) {
        for(int col=0; col<LARGURA; col++) {
            int sum = 0;
            for(int i=-pad; i<=pad; i++) {
                for(int j=-pad; j<=pad; j++) {
                    int y = row + i, x = col + j;
                    if(y < 0) y = 0;
                    if(y >= ALTURA) y = ALTURA - 1;
                    if(x < 0) x = 0;
                    if(x >= LARGURA) x = LARGURA - 1;
                    sum += image[y][x] * kernel[i+pad][j+pad];
                }
            }
            output[row][col] = sum;
        }
    }
}

void sobel_edge_detection(int image[ALTURA][LARGURA], float grad_mag[ALTURA][LARGURA], float grad_dir[ALTURA][LARGURA]) {
    int gx_kernel[3][3] = {
        {-1,0,1},
        {-2,0,2},
        {-1,0,1}
    };
    int gy_kernel[3][3] = {
        {-1,-2,-1},
        {0,0,0},
        {1,2,1}
    };

    int grad_x[ALTURA][LARGURA], grad_y[ALTURA][LARGURA];
    convolution(image, gx_kernel, grad_x);
    convolution(image, gy_kernel, grad_y);

    float max_val = 0.0f;
    for(int i=0;i<ALTURA;i++)for(int j=0;j<LARGURA;j++) {
        grad_mag[i][j] = sqrt(grad_x[i][j]*grad_x[i][j] + grad_y[i][j]*grad_y[i][j]);
        if(grad_mag[i][j]>max_val)
            max_val = grad_mag[i][j];
    }
    // Normaliza para 0..255
    for(int i=0;i<ALTURA;i++)for(int j=0;j<LARGURA;j++)
        grad_mag[i][j] = grad_mag[i][j] * 255.0f / max_val;
    
    for(int i=0;i<ALTURA;i++)for(int j=0;j<LARGURA;j++)
        grad_dir[i][j] = atan2f(grad_y[i][j], grad_x[i][j]) * 180.0f /PI;

    printf("cheguei no sobel\n");
}

void non_max_suppression(float grad_mag[ALTURA][LARGURA], float grad_dir[ALTURA][LARGURA], int output[ALTURA][LARGURA]) {
    for(int row=1; row<ALTURA-1; row++) {
        for(int col=1; col<LARGURA-1; col++) {
            float direction = grad_dir[row][col];
            if(direction<0) direction += 180;
            float mag = grad_mag[row][col];
            float before_pixel = 0, after_pixel = 0;

            if((0 <= direction && direction < 22.5) || (157.5 <= direction && direction <= 180)) {
                before_pixel = grad_mag[row][col-1];
                after_pixel  = grad_mag[row][col+1];
            } else if((22.5 <= direction && direction < 67.5)) {
                before_pixel = grad_mag[row+1][col-1];
                after_pixel = grad_mag[row-1][col+1];
            } else if((67.5 <= direction && direction < 112.5)) {
                before_pixel = grad_mag[row-1][col];
                after_pixel = grad_mag[row+1][col];
            } else {
                before_pixel = grad_mag[row-1][col-1];
                after_pixel = grad_mag[row+1][col+1];
            }
            if(mag >= before_pixel && mag >= after_pixel)
                output[row][col] = (int)mag;
            else
                output[row][col] = 0;
        }
    }
    printf("cheguei aqui na supressao\n");
}

void threshold(int img[ALTURA][LARGURA], int low, int high, int output[ALTURA][LARGURA]) {
    for(int i=0;i<ALTURA;i++)
        for(int j=0;j<LARGURA;j++) {
            if(img[i][j] >= high)
                output[i][j] = 255;
            else if(img[i][j] >= low)
                output[i][j] = 75;
            else
                output[i][j] = 0;
        }
    printf("cheguei aqui no threshold\n");
}

void hysteresis(int img[ALTURA][LARGURA]) {
    // 4 passes: top-down, bottom-up, left-right, right-left
    for(int pass=0; pass<4; pass++) {
        int rs = (pass&1)==0 ? 1 : ALTURA-2, re = (pass&1)==0 ? ALTURA-1 : 0, r_step = (rs<re?1:-1);
        int cs = (pass&2)==0 ? 1 : LARGURA-2, ce = (pass&2)==0 ? LARGURA-1 : 0, c_step = (cs?1:-1);
        for(int row=rs; row!=re; row+=r_step)
            for(int col=cs; col!=ce; col+=c_step)
                if(img[row][col]==75) {
                    for(int di=-1;di<=1;di++)for(int dj=-1;dj<=1;dj++) {
                        if(di==0&&dj==0) continue;
                        if(img[row+di][col+dj]==255) {
                            img[row][col]=255; break;
                        }
                    }
                    if(img[row][col]!=255)
                        img[row][col]=0;
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
    ler_pgm("./mapaP2.pgm");

    printf("Aplicando filtro gaussiano...\n");
    filtro_gaussiano();

    printf("Calculando gradientes...\n");
    //calc_gradiente();
    sobel_edge_detection(img, mag, dir);

    printf("Supressão não-máxima...\n");
    //supr_nao_max();
    non_max_suppression(mag, dir, img);

    printf("Realizando limiarização com estereses 30 e 75...\n");
    //limiarizacao_histerese(30, 75);
    threshold(img, 30, 75, output);
    hysteresis(output);

    printf("Sobrescrevendo e salvando imagem...\n");
    salvar_pgm("output.pgm", output);
}