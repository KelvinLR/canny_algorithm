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

int img[89][89] = {0};
int dir[89][89] = {0};
int bordas[89][89] = {0};
int largura, altura;

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
        }
    }

    salvar_pgm("gauss.pgm", temp); // Usa o buffer temporário igual ao seu modelo
}

void calc_gradiente() {
    unsigned char temp_mag[89][89] = {0};
    int sobelGx[3][3] = {{-1,0,1},{-2,0,2},{-1,0,1}};
    int sobelGy[3][3] = {{-1,-2,-1},{0,0,0},{1,2,1}};

    for(int i = 1; i < 88; i++) {
        for(int j = 1; j < 88; j++) {
            float gx = 0, gy = 0;
            for(int ki = -1; ki <= 1; ki++)
                for(int kj = -1; kj <= 1; kj++) {
                    int pixel = img[i + ki][j + kj];
                    gx += (float)pixel * sobelGx[ki + 1][kj + 1];
                    gy += (float)pixel * sobelGy[ki + 1][kj + 1];
                }
            float mag = sqrt(gx*gx + gy*gy);
            if (mag > 255.0f) mag = 255.0f;
            if (mag < 0.0f) mag = 0.0f;
            temp_mag[i][j] = (unsigned char)mag; // truncamento igual a uint8

            float angulo = atan2(gy, gx) * 180.0f / PI;
            if (angulo < 0) angulo += 180.0f;
            if (angulo < 22.5 || angulo >= 157.5)
                dir[i][j] = 0;
            else if (angulo < 67.5)
                dir[i][j] = 45;
            else if (angulo < 112.5)
                dir[i][j] = 90;
            else
                dir[i][j] = 135;
        }
    }
    for(int i = 0; i < 89; i++)
        for(int j = 0; j < 89; j++)
            img[i][j] = temp_mag[i][j];

}

void supr_nao_max() {
    for (int i = 1; i < altura-1; i++) {
        for (int j = 1; j < largura-1; j++) {
            unsigned char mag = img[i][j];
            int direcao = dir[i][j];
            unsigned char n1, n2;
            switch (direcao) {
                case 0:   n1 = img[i][j-1];   n2 = img[i][j+1];   break;
                case 45:  n1 = img[i-1][j+1]; n2 = img[i+1][j-1]; break;
                case 90:  n1 = img[i-1][j];   n2 = img[i+1][j];   break;
                case 135: n1 = img[i-1][j-1]; n2 = img[i+1][j+1]; break;
                default:  n1 = n2 = 0;
            }
            bordas[i][j] = (mag >= n1 && mag >= n2) ? mag : 0;
        }
    }
}

void limiarizacao_histerese(int limiar_baixo, int limiar_alto) {
    unsigned char forte = 255;
    unsigned char fraco = 75;

    for (int i = 0; i < altura; i++)
        for (int j = 0; j < largura; j++)
            if (bordas[i][j] >= limiar_alto)
                bordas[i][j] = forte;
            else if (bordas[i][j] >= limiar_baixo)
                bordas[i][j] = fraco;
            else
                bordas[i][j] = 0;

    int mudou = 1;
    while (mudou) {
        mudou = 0;
        for (int i = 1; i < altura-1; i++) {
            for (int j = 1; j < largura-1; j++) {
                if (bordas[i][j] == fraco) {
                    for (int di = -1; di <= 1; di++)
                        for (int dj = -1; dj <= 1; dj++) {
                            if (bordas[i+di][j+dj] == forte) {
                                bordas[i][j] = forte;
                                mudou = 1;
                                goto next;
                            }
                        }
                    next: ;
                }
            }
        }
    }

    for (int i = 0; i < altura; i++)
        for (int j = 0; j < largura; j++)
            if (bordas[i][j] == fraco)
                bordas[i][j] = 0;
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
    ler_pgm("./musga.pgm");

    printf("Aplicando filtro gaussiano...\n");
    filtro_gaussiano();

    printf("Calculando gradientes...\n");
    calc_gradiente();

    printf("Supressão não-máxima...\n");
    supr_nao_max();

    printf("Realizando limiarização com estereses 30 e 75...\n");
    limiarizacao_histerese(30, 75);

    printf("Sobrescrevendo e salvando imagem...\n");
    salvar_pgm("output.pgm", bordas);
}