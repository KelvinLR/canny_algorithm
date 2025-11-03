/** ALGORITMO CANNY PARA DETECÇÃO DE BORDAS
 * 
 * @file    main.c
 * @brief   Implementação from scratch do algoritmo Canny de detecção de bordas.
 * 
 * PASSO A PASSO
 * \_ 1) aplicação de filtro gaussiano 2D
 * \_ 2) encontrar gradiente utilizando Sobel
 * \_ 3) supressão não-máxima
 * \_ 4) limiarização com histerese
 * 
 * @author  Kelvin de Lima Rodrigues
 * @author  Marcelo Antônio Dantas Filho
 * 
 * @date    29/10/2025
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265
#define ALTURA 89
#define LARGURA 89


uint64_t img[89][89]; //< Matriz que recebe a imagem de entrada
double mag[89][89]; //< Matriz para armazenar a magnitude do gradiente
double dir[89][89]; //< Matriz para armazenar a direção do gradiente
uint64_t output[89][89]; //< Matriz que receberá a imagem de saída

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
void salvar_pgm(const char* nome_arquivo, int matriz[89][89]) {
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

/**
 * @brief   copia uma matriz para outra (89x89)
 * @param   input   matriz origem
 * @param   output  matriz destino
 */
void imgcopy(const int input[ALTURA][LARGURA], int output[ALTURA][LARGURA]) {
    for(int i = 0; i < ALTURA; ++i)
        for(int j = 0; j < LARGURA; ++j)
            output[i][j] = input[i][j];
}

/**
 * @brief   aplica o Gaussian Blur para redução de ruídos na imagem
 */
void filtro_gaussiano() {
    int temp[89][89]; //< buffer temporário

    // kernel padrão 5x5 pré normalizado com a soma dos elementos da matriz
    double kernel[5][5] = {
        {2.0f/159.0f, 4.0f/159.0f, 5.0f/159.0f, 4.0f/159.0f, 2.0f/159.0f},
        {4.0f/159.0f, 9.0f/159.0f, 12.0f/159.0f, 9.0f/159.0f, 4.0f/159.0f},
        {5.0f/159.0f, 12.0f/159.0f, 15.0f/159.0f, 12.0f/159.0f, 5.0f/159.0f},
        {4.0f/159.0f, 9.0f/159.0f, 12.0f/159.0f, 9.0f/159.0f, 4.0f/159.0f},
        {2.0f/159.0f, 4.0f/159.0f, 5.0f/159.0f, 4.0f/159.0f, 2.0f/159.0f}
    };

    int k = 2; //< variável para calcular o deslocamento em torno do pixel central
    int i, j, m, n;
    float sum; //< acumulador
    int row_idx, col_idx; //<índices de linhas e colunas

    // laço para percorrer cada pixel da imagem de saída
    for(i = 0; i < altura; i++) {
        for(j = 0; j < largura; j++) {

            sum = 0.0f; // zera o acumulador sum p/ esse pixel

            // aplicação do kernel na vizinhança do pixel atual
            for(m = -k; m <= k; m++) {
                for(n = -k; n <= k; n++) {

                    // calcula o indice da linha vizinha ao kernel
                    row_idx = i + m;
                    // se estiver fora ajusta para a borda mais proxima
                    if(row_idx < 0)
                        row_idx = 0;

                    else if(row_idx >= altura)
                        row_idx = altura - 1;

                    // mesmo calculo mas para coluna
                    col_idx = j + n;
                    if(col_idx < 0)
                        col_idx = 0;

                    else if(col_idx >= largura)
                        col_idx = largura - 1;

                    // multiplica o pixel correspondente pelo valor do kernel
                    // acumula o resultado no acumulador sum
                    sum += img[row_idx][col_idx] * kernel[m + k][n + k];
                }
            }

            // limita os valores para ficar entre 0 e 255
            if(sum < 0.0f)
                sum = 0.0f;

            if(sum > 255.0f)
                sum = 255.0f;
            // passagem do valor (typecasting) inteiro para a matriz temporária
            temp[i][j] = (int)sum;

        }
    }
    // copia a imagem em temp para a matriz img
    imgcopy(temp, img);
}

/**
 * @brief   realiza a convolução do kernel com a imagem
 * 
 * @param   image   imagem de entrada
 * @param   kernel  kernel 3x3
 * @param   output  matriz resultado da convolução
 */
void convolution(int image[ALTURA][LARGURA], int kernel[3][3], int output[ALTURA][LARGURA]) {
    int pad = 1; //< padding para kernel padrão 3x3 -> qtd de pixels da margem

    for(int row=0; row<ALTURA; row++) {
        for(int col=0; col<LARGURA; col++) {
            int sum = 0; //< acumulador

            for(int i=-pad; i<=pad; i++) {
                for(int j=-pad; j<=pad; j++) {

                    int y = row + i, x = col + j;
                    // condicional para verificar as bordas, caso
                    // os vizinhos estejam fora dos limites da img (ultrapassando a borda),
                    // a posição será ignorada
                    if(y < 0 || y >= ALTURA || x < 0 || x >= LARGURA)
                        continue;
                    // preenche a variável sum com o produto do valor do pixel com o Kernel
                    sum += image[y][x] * kernel[i+pad][j+pad];
                }
            }
            // popula a matriz de saída com o resultado da convolucao acumulado em sum
            output[row][col] = sum;
        }
    }
}

/**
 * @brief   Calcula os gradientes usando os operadores de Sobel
 * 
 * @param   image       imagem de entrada
 * @param   grad_mag    matriz de magnitudes do gradiente
 * @param   grad_dir    matriz de direção dos gradientes (em graus)
 */
void sobel_edge_detection(int image[ALTURA][LARGURA], double grad_mag[ALTURA][LARGURA], double grad_dir[ALTURA][LARGURA]) {
    //< gx e gy -> operadores padrão de sobel para fazer a varredura e detecção das bordas
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

    //< matrizes grad_x e grad_y que serão a matriz de sáida da função convolution
    int grad_x[ALTURA][LARGURA], grad_y[ALTURA][LARGURA];

    // realização da convolução das funções, onde os kernels Gx e Gy são chamados
    convolution(image, gx_kernel, grad_x);
    convolution(image, gy_kernel, grad_y);

    double max_val = 0.0f; //< valor máximo

    for(int i=0;i<ALTURA;i++){
        for(int j=0;j<LARGURA;j++) {

            // magnitude do gradiente determinada pelo cálculo da raiz dos quadrados de cada pixel do gradiente
            grad_mag[i][j] = sqrt(grad_x[i][j]*grad_x[i][j] + grad_y[i][j]*grad_y[i][j]);

            // caso o valor do índice [i][j] da matriz que armazena  a magnitude dos gradientes seja maior que 0
            // o valor máximo recebe o valor presente no índice [i][j]
            if(grad_mag[i][j]>max_val)
                max_val = grad_mag[i][j];
        }
    }

    // faz a normalização para manter os valores de 0 a 255
    // percorre a matriz da magnitude dos gradientes
    for(int i=0;i<ALTURA;i++) {
        for(int j=0;j<LARGURA;j++) {

            grad_mag[i][j] = grad_mag[i][j] * 255.0f / max_val;

            // converte o resultado para 3 casas decimais
            grad_mag[i][j] = (long long)(grad_mag[i][j] * 1000) / 1000.0;
        }
    }

    // percorre a matriz da direção dos gradientes
    for(int i=0;i<ALTURA;i++) {
        for(int j=0;j<LARGURA;j++) {

            // cálculo da direção dos gradientes e conversão para graus
            grad_dir[i][j] = atan2f(grad_y[i][j], grad_x[i][j]) * (180.0f /PI) + 180;

            // converte o resultado para 3 casas decimais
            grad_dir[i][j] = (long long)(grad_dir[i][j] * 1000) / 1000.0;
        }
    }
}

/**
 * @brief   realiza a supressão não-máxima na matriz de magnitudes do gradiente
 * 
 * mantém apenas os pixels que correspondem a máximos locais ao longo da direção do gradiente
 * 
 * @param   grad_mag    magnitude do gradiente
 * @param   grad_dir    direção do gradiente
 * @param   output      matriz de saída após supressão
 */
void non_max_suppression(double grad_mag[ALTURA][LARGURA], double grad_dir[ALTURA][LARGURA], int output[ALTURA][LARGURA]) {
    // percorre cada linhas e colunas de grad_mag.
    for (int row = 0; row < ALTURA; row++) {
        for (int col = 0; col < LARGURA; col++) {

            // ignora as bordas da imagem, pois não possui vizinhos complexos.
            if(row == 0 || col == 0 || row == ALTURA-1 || col == LARGURA-1) {
                output[row][col] = 0;
                continue;
            }

            double direction = grad_dir[row][col]; //< direção do gradiente no pixel atual (em graus)
            double mag = grad_mag[row][col]; //< magnitude do gradiente no pixel atual

            int by, bx, ay, ax; //< indices dos vizinhos antes e depois ao longo da direção do gradiente
            double before_pixel = 0, after_pixel = 0; //< magnitude dos vizinhos

            // determina quais vizinhos checar de acordo com base no angulo da direcao do grad
            // direções agrupadas p comparar apenas vizinhos na direção aproximada do grad
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

            // obtem as magnitudes dos pixels vizinhos ao longo da direção do gradiente
            before_pixel = grad_mag[by][bx];
            after_pixel = grad_mag[ay][ax];

            // realizando a supressao mantendo o pixel se ele for maior ou igual aos seus vizinhos
            if (mag >= before_pixel && mag >= after_pixel)
                output[row][col] = (int)floor(mag);

            // descarta valor caso nao se enquadre na condicao e substitui por 0
            else
                output[row][col] = 0; 
        }
    }
}

/**
 * @brief   aplica a limiarização com pixels fortes e fracos e descarta o resto
 * 
 * Classifica pixels:
 * 
 * - >= high -> forte (255);
 * 
 * - entre low e high -> fraco;
 * 
 * - resto -> 0
 * 
 * @param   img     imagem de entrada
 * @param   low     limiar baixo
 * @param   high    limiar alto
 * @param   weak    valor atribuído aos pixels fracos
 * @param   output  matriz resultante da limiarização
 */
void threshold(int img[ALTURA][LARGURA], int low, int high, int weak, int output[ALTURA][LARGURA]) {
    int strong = 255; //< valor do pixel forte

    // percorre a matriz atribuindo os rótulos conforme as faixas definidas.
    for(int i=0;i<ALTURA;i++) {
        for(int j=0;j<LARGURA;j++) {

            if(img[i][j] >= high) {
                output[i][j] = strong; // pixel forte
            }

            else if(img[i][j] <= high && img[i][j] >= low)
                output[i][j] = weak;   // pixel fraco
            // pixels abaixo de low nao serao alterados
        }
    }

}

/**
 * @brief   verifica se existe algum vizinho 3x3 com valor de pixel forte (255)
 * 
 * @param   img    imagem de entrada
 * @param   l   coordenada da linha
 * @param   c   coordenada da coluna
 * 
 * @returns 1, se encontrar;
 * 
 * 0, se não encontrar.
 */
static int has_strong_neighbor(const int img[ALTURA][LARGURA], int l, int c) {
    int strong = 255;
    const int movs[] = {-1, 0, 1};

    for(int i = 0; i < 3; ++i) {
        int nl = l + movs[i];

        if (nl < 0 || nl >= ALTURA) continue; // limita aos limites da imagem

        for (int j = 0; j < 3; ++j) {
            int nc = c + movs[j];

            if (nc < 0 || nc >= LARGURA) continue;

            if (img[nl][nc] == strong) return 1;  // retorna que encontrou vizinho forte
        }
    }
    return 0;
}

/**
 * @brief   faz o rastreamento de bordas por histerese,
 * fazendo com que pixels fracos conectados a fortes sejam postos em evidencia; o resto é zerado.
 * 
 * @param   img     imagem de entrada
 * @param   weak    valor do pixel fraco
 */
void hysteresis(int img[ALTURA][LARGURA], int weak) {
    // faz cópias da imagem para varredura em direções diferentes
    int top_bottom[ALTURA][LARGURA], bottom_up[ALTURA][LARGURA];
    int left_right[ALTURA][LARGURA], right_left[ALTURA][LARGURA];

    imgcopy(img, top_bottom);
    imgcopy(img, bottom_up);
    imgcopy(img, left_right);
    imgcopy(img, right_left);

    int strong = 255; //< valor atribuído ao pixel forte

    // 1: de cima para baixo e esquerda para direita
    int i = 0, j = 0;
    for (i = 1; i < ALTURA; ++i)
        for (j = 1; j < LARGURA; ++j)

            if (top_bottom[i][j] == weak) {
                if (has_strong_neighbor(top_bottom, i, j)) {
                    top_bottom[i][j] = strong; // promove o pixel fraco a forte caso tenha um vizinho forte

                } else {
                    top_bottom[i][j] = 0; // caso contrario zera o pixel e isso se repete nas 4 varreduras
                }
            };

    // 2: de baixo para cima e direita para esquerda
    for (i = ALTURA - 2; i >= 1; --i)
        for (j = LARGURA - 2; j >= 1; --j)
            if (bottom_up[i][j] == weak) {
                if (has_strong_neighbor(bottom_up, i, j)) {
                    bottom_up[i][j] = strong;
                } else {
                    bottom_up[i][j] = 0;
                }
            };

    // 3: da esquerda para direita e de baixo para cima
    for (i = 1; i < ALTURA - 1; ++i)
        for (j = LARGURA - 2; j >= 1; --j)
            if (right_left[i][j] == weak) {
                if (has_strong_neighbor(right_left, i, j)) {
                    right_left[i][j] = strong;
                } else {
                    right_left[i][j] = 0;
                }
            };

    // 4: da direita para esquerda e de cima para baixo
    for (i = ALTURA - 2; i >= 1; --i)
        for (j = 1; j < LARGURA - 1; ++j)
            if (left_right[i][j] == weak) {
                if (has_strong_neighbor(left_right, i, j)) {
                    left_right[i][j] = strong;
                } else {
                    left_right[i][j] = 0;
                }
            }

    // qualquer pixel que foi posto em evidencia nos passos anteriores em pelo menos uma passagem
    // vira um pixel forte (vai pra 255)
    for (i = 0; i < ALTURA; ++i) {
        for (j = 0; j < LARGURA; ++j) {

            int soma = top_bottom[i][j] + bottom_up[i][j] + left_right[i][j] +
                       right_left[i][j];

            img[i][j] = (soma > 255) ? 255 : soma;
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
    filtro_gaussiano();
    salvar_pgm("gauss.pgm", img);

    printf("calculando gradientes...\n");
    sobel_edge_detection(img, mag, dir);

    printf("supr nao-maxima...\n");
    non_max_suppression(mag, dir, img);
    salvar_pgm("supr.pgm", img);

    printf("fazendo a limiarização...\n");
    int weak = 75;
    threshold(img, 5, 20, weak, output);
    salvar_pgm("thresh.pgm", output);

    printf("finalizando com histerese...\n");
    hysteresis(output, weak);

    printf("salvando img...\n");
    salvar_pgm("output.pgm", output);
}
