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

/**
 * @brief   copia uma matriz para outra (89x89)
 * @param   input   matriz origem
 * @param   output  matriz destino
 */
void imgcopy(const uint8_t input[ALTURA][LARGURA], uint8_t output[ALTURA][LARGURA]) {
    for(int i = 0; i < ALTURA; ++i)
        for(int j = 0; j < LARGURA; ++j)
            output[i][j] = input[i][j];
}

/**
 * @brief   aplica o Gaussian Blur para redução de ruídos na imagem
 */
void filtro_gaussiano() {
    uint8_t temp[1][89]; //< buffer temporário

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
void convolution(uint8_t image[ALTURA][LARGURA], uint8_t kernel[3][3], uint8_t output[ALTURA][LARGURA]) {
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
void sobel_edge_detection(uint8_t img[89][89]) {
    int grad_x[89][89], grad_y[89][89];
    double mag[89][89];
    double max_val = 0.0;

    int gx_kernel[3][3] = {
        {-1, 0, 1},
        {-2, 0, 2},
        {-1, 0, 1}
    };
    int gy_kernel[3][3] = {
        {1, 2, 1},
        {0, 0, 0},
        {-1, -2, -1}
    };

    // Aplica convolução para grad_x e grad_y
    for (int i = 0; i < 89; i++) {
        for (int j = 0; j < 89; j++) {
            int gx = 0, gy = 0;
            // Bordas: igual ao convolution com "clamping" para i+ki e j+kj dentro dos limites
            for (int ki = -1; ki <= 1; ki++) {
                int ni = i + ki;
                if (ni < 0) ni = 0;
                if (ni > 88) ni = 88;
                for (int kj = -1; kj <= 1; kj++) {
                    int nj = j + kj;
                    if (nj < 0) nj = 0;
                    if (nj > 88) nj = 88;
                    gx += img[ni][nj] * gx_kernel[ki+1][kj+1];
                    gy += img[ni][nj] * gy_kernel[ki+1][kj+1];
                }
            }
            grad_x[i][j] = gx;
            grad_y[i][j] = gy;
            mag[i][j] = sqrt((double)gx*gx + (double)gy*gy);
            if (mag[i][j] > max_val)
                max_val = mag[i][j];
        }
    }
    // Normaliza e copia para img
    for (int i = 0; i < 89; i++) {
        for (int j = 0; j < 89; j++) {
            //
            img[i][j] = (uint8_t)(mag[i][j] * 255.0 / max_val + 0.5); // +0.5 garante arredondamento
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
void non_max_suppression(uint8_t img[89][89]) {
    uint8_t result[89][89];

    for (int i = 1; i < 88; i++) {
        for (int j = 1; j < 88; j++) {
            uint8_t mag = img[i][j];

            // Checa se é máximo horizontal
            uint8_t before_h = img[i][j-1];
            uint8_t after_h  = img[i][j+1];

            // Checa se é máximo vertical
            uint8_t before_v = img[i-1][j];
            uint8_t after_v  = img[i+1][j];

            // Mantém o pixel se for máximo em qualquer direção
            if ((mag >= before_h && mag >= after_h) ||
                (mag >= before_v && mag >= after_v)) {
                result[i][j] = mag;
            } else {
                result[i][j] = 0;
            }
        }
    }
    // Copia resultado para buffer principal
    for (int i = 1; i < 88; i++)
        for (int j = 1; j < 88; j++)
            img[i][j] = result[i][j];
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
void threshold(uint8_t img[89][89], uint8_t low, uint8_t high, uint8_t weak) {
    for (int i = 0; i < 89; i++) {
        for (int j = 0; j < 89; j++) {
            if (img[i][j] >= high)
                img[i][j] = 255; // forte
            else if (img[i][j] >= low)
                img[i][j] = weak; // fraco
            else
                img[i][j] = 0;
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
static int has_strong_neighbor(const uint8_t img[ALTURA][LARGURA], int l, int c) {
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
void hysteresis(uint8_t img[89][89], uint8_t weak) {
    // 1a passada: de cima para baixo
    for (int i = 1; i < 88; i++) {
        for (int j = 1; j < 88; j++) {
            if (img[i][j] == weak) {
                if (img[i-1][j] == 255 || img[i+1][j] == 255 || img[i][j-1] == 255 || img[i][j+1] == 255)
                    img[i][j] = 255;
                else
                    img[i][j] = 0;
            }
        }
    }
    // 2a passada: de baixo para cima (garante propagação)
    for (int i = 87; i >= 1; i--) {
        for (int j = 87; j >= 1; j--) {
            if (img[i][j] == weak) {
                if (img[i-1][j] == 255 || img[i+1][j] == 255 || img[i][j-1] == 255 || img[i][j+1] == 255)
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
    filtro_gaussiano();
    salvar_pgm("gauss.pgm", img);

    printf("calculando gradientes...\n");
    sobel_edge_detection(img);
    salvar_pgm("sobel.pgm", img);

    printf("supr nao-maxima...\n");
    non_max_suppression(img);
    salvar_pgm("supr.pgm", img);

    printf("fazendo a limiarização...\n");
    int weak = 75;
    threshold(img, 5, 20, weak);
    salvar_pgm("thresh.pgm", img);

    printf("finalizando com histerese...\n");
    hysteresis(img, weak);

    printf("salvando img...\n");
    salvar_pgm("output.pgm", img);
}