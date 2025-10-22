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

int img[89][89];
int dir[89][89];
int bordas[89][89];
int largura, altura;
//se declara-las dentro da função de ler pgm, o valor delas vai ser perdido depois que a função terminar de executar

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

void filtro_gaussiano(int x, int y) {
    //largura e altura tem que ser x e y ou nenhum parâmetro deve ser passado e usamos as variaveis globais
    int i, j;
    int kernel[5][5] = {
        {2, 4, 5, 4, 2},
        {4, 9, 12, 9, 4},
        {5, 12, 15, 12, 5},
        {4, 9, 12, 9, 4},
        {2, 4, 5, 4, 2}
    };
    int sum = 159;

    for(i = 0; i < x; i++){
        for(j = 0; j < y; j++){
            int val = 0;
            for (int ky = -2; ky <= 2; ky++) {
                for (int kx = -2; kx <= 2; kx++) {
                    val += img[y + ky][x + kx] * kernel[ky + 2][kx + 2];
                }
            }
            img[y][x] = val / sum;
        }
    }
}

void calc_gradiente() {
    int direcao, magnitude, i, j;
    int sobelGx[3][3] = {
        {-1, 0, 1},
        {-2, 0, 2},
        {-1, 0, 1},
    };
    int sobelGy[3][3] = {
        {1, 2, 1},
        {0, 0, 0},
        {-1, -2, -1},
    };

    for(int i = 1; i < 89 - 1; i++) {
        for(int j = 1; j < 89 - 1; j++) {
            int soma_eixo_x = 0;
            int soma_eixo_y = 0;

            for(int ki = -1; ki <= 1; ki++) {
                for(int kj = -1; kj <= 1; kj++) {
                    int pixel = img[i + ki][j + kj];
                    soma_eixo_x += pixel * sobelGx[ki + 1][kj + 1];
                    soma_eixo_y += pixel * sobelGy[ki + 1][kj + 1];
                }
            }

            magnitude = (int)sqrt(soma_eixo_x * soma_eixo_x + soma_eixo_y * soma_eixo_y);
            img[i][j] = magnitude;

            //direção do gradiente (em graus)
            float angulo = atan2(soma_eixo_y, soma_eixo_x) * 180/PI;
            if (angulo < 0) angulo += 180;

            if (angulo<22.5 || angulo>=157.5)
            {
                direcao = 0;
            }
            else if (angulo<67.5)
            {
                direcao = 45;
            }
            else if (angulo < 112.5)
            {
                direcao = 90;
            }
            else direcao = 135;

            dir[i][j] = direcao;
        }
    }
}

void supr_nao_max() {
    for (int i = 1; i < altura; i++){
        for (int j = 1; j < largura; i++){
            int mag = img[i][j];
            int direcao = dir[i][j];
            int n1, n2;

            switch (direcao)
            {
            case 0:
                n1 = img[i][j-1];
                n2 = img[i][j+1];
                break;
            case 45:
                n1 = img[i-1][j+1];
                n2 = img[i+1][j-1];
                break;
            case 90:
                n1 = img[i-1][j];
                n2 = img[i+1][j];
                break;
            case 135:
                n1 = img[i-1][j-1];
                n2 = img[i+1][j+1];
                break;
            default:
                n1 = n2 = 0;
            }

            if(mag >= n1 && mag >=  n2)
            bordas[i][j] = mag > 255 ? 255 : mag;
            else
            bordas[i][j] = 0;
        }
    }
}

void limiarizacao_histerese() {}

int main() {

    ler_pgm("./exemplo.pgm");
    calc_gradiente();
    supr_nao_max();
    limiarizacao_histerese();
}