#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <math.h>

#define PI 3.14159265

// tile completo a ser recebido
#define TILE 24
// borda de contexto
#define BORDER_TILE 4
// tile util enviado pela serial
#define TILE_UTIL (TILE - 2 * BORDER_TILE)

UART_HandleTypeDef huart2;

// tile recebido!
uint8_t tile[TILE][TILE];

// matrizes de direçao e magnitude do grady
float grad_mag[TILE][TILE];
float grad_dir[TILE][TILE];

void imgcopy_uint8(const uint8_t src[TILE][TILE], uint8_t dst[TILE][TILE]) {
   for (int i = 0; i < TILE; ++i)
       for (int j = 0; j < TILE; ++j)
           dst[i][j] = src[i][j];
}

void filtro_gaussiano(uint8_t img[TILE][TILE]) {
   static const float kernel[5][5] = {
           {2.0f/159.0f, 4.0f/159.0f, 5.0f/159.0f, 4.0f/159.0f, 2.0f/159.0f},
           {4.0f/159.0f, 9.0f/159.0f,12.0f/159.0f, 9.0f/159.0f, 4.0f/159.0f},
           {5.0f/159.0f,12.0f/159.0f,15.0f/159.0f,12.0f/159.0f, 5.0f/159.0f},
           {4.0f/159.0f, 9.0f/159.0f,12.0f/159.0f, 9.0f/159.0f, 4.0f/159.0f},
           {2.0f/159.0f, 4.0f/159.0f, 5.0f/159.0f, 4.0f/159.0f, 2.0f/159.0f}
   };
   uint8_t temp[TILE][TILE];
   for (int i = 0; i < TILE; ++i) {
       for (int j = 0; j < TILE; ++j) {
           float sum = 0.0f;
           for (int m = -2; m <= 2; ++m) {
               int y = i + m;
               if (y < 0) y = 0;
               else if (y >= TILE) y = TILE - 1;
               for (int n = -2; n <= 2; ++n) {
                   int x = j + n;
                   if (x < 0) x = 0;
                   else if (x >= TILE) x = TILE - 1;
                   sum += img[y][x] * kernel[m + 2][n + 2];
               }
           }
           int val = (int)sum;
           if (val < 0) val = 0;
           else if (val > 255) val = 255;
           temp[i][j] = (uint8_t)val;
       }
   }
   imgcopy_uint8(temp, img);
}

void convolution(uint8_t image[TILE][TILE], int8_t kernel[3][3], int16_t output[TILE][TILE]) {
   int pad = 1; // padding para kernel padrão 3x3 -> qtd de pixels da margem
   for(int row=0; row<TILE; row++) {
       for(int col=0; col<TILE; col++) {
           int sum = 0;
           for(int i=-pad; i<=pad; i++) {
               for(int j=-pad; j<=pad; j++) {
                   int y = row + i, x = col + j;
                   // condicional para verificar as bordas, caso
                   // os vizinhos estejam fora dos limites da img (ultrapassando a borda),
                   // a posição será ignorada
                   if(y < 0 || y >= TILE || x < 0 || x >= TILE)
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

void sobel_edge_detection(uint8_t image[TILE][TILE], float grad_mag[TILE][TILE], float grad_dir[TILE][TILE]) {
   // gx e gy são os operadores padrão de sobel para fazer a varredura e detecção das bordas
   int8_t gx_kernel[3][3] = {
       {-1,0,1},
       {-2,0,2},
       {-1,0,1}
   };
   int8_t gy_kernel[3][3] = {
       {1,2,1},
       {0,0,0},
       {-1,-2,-1},
   };
   // matrizes grad_x e grad_y que serão a matriz de sáida da função convolution
   int16_t grad_x[TILE][TILE], grad_y[TILE][TILE];
   // realização da convolução das funções, onde os kernels Gx e Gy são chamados
   convolution(image, gx_kernel, grad_x);
   convolution(image, gy_kernel, grad_y);
   double max_val = 0.0f;
   for(int i=0;i<TILE;i++){
       for(int j=0;j<TILE;j++) {
           grad_mag[i][j] = sqrt(grad_x[i][j]*grad_x[i][j] + grad_y[i][j]*grad_y[i][j]);
           if(grad_mag[i][j]>max_val)
               max_val = grad_mag[i][j];
       }
   }
   for(int i=0;i<TILE;i++) {
       for(int j=0;j<TILE;j++) {
           grad_mag[i][j] = grad_mag[i][j] * 255.0f / max_val;
           grad_mag[i][j] = (long long)(grad_mag[i][j] * 1000) / 1000.0;
       }
   }
   for(int i=0;i<TILE;i++) {
       for(int j=0;j<TILE;j++) {
           grad_dir[i][j] = atan2f(grad_y[i][j], grad_x[i][j]) * (180.0f /PI) + 180;
           grad_dir[i][j] = (long long)(grad_dir[i][j] * 1000) / 1000.0;
       }
   }
}

void non_max_suppression(float grad_mag[TILE][TILE], float grad_dir[TILE][TILE], uint8_t output[TILE][TILE]) {
   for (int row = 0; row < TILE; row++) {
       for (int col = 0; col < TILE; col++) {
           if(row == 0 || col == 0 || row == TILE-1 || col == TILE-1) {
               output[row][col] = 0;
               continue;
           }
           float direction = grad_dir[row][col]; // direcao do gradiente no pixel atual (graus)
           float mag = grad_mag[row][col]; // magnitude do gradiente no pixel atual
           int by, bx, ay, ax; // indices dos vizinhos antes e depois ao longo da direção do gradiente
           double before_pixel = 0, after_pixel = 0; // magnitude dos vizinhos
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
           else
               output[row][col] = 0; // descarta valor caso nao se enquadre na condicao e substitui por 0
       }
   }
}

void threshold(uint8_t img[TILE][TILE], int low, int high, int weak, uint8_t output[TILE][TILE]) {
   int strong = 255;
   for(int i=0;i<TILE;i++) {
       for(int j=0;j<TILE;j++) {
           if(img[i][j] >= high) {
               output[i][j] = strong; // pixel forte
           }
           else if(img[i][j] <= high && img[i][j] >= low)
               output[i][j] = weak;   // pixel fraco
           // pixels abaixo de low nao serao alterados
       }
   }
}

static int has_strong_neighbor(const uint8_t img[TILE][TILE], int l, int c) {
   int strong = 255;
   const int movs[] = {-1, 0, 1};
   for(int i = 0; i < 3; ++i) {
       int nl = l + movs[i];
       if (nl < 0 || nl >= TILE) continue; // limita aos limites da imagem
       for (int j = 0; j < 3; ++j) {
           int nc = c + movs[j];
           if (nc < 0 || nc >= TILE) continue;
           if (img[nl][nc] == strong) return 1;  // retorna que encontrou vizinho forte
       }
   }
   return 0;
}


void hysteresis(uint8_t img[TILE][TILE], int weak) {
   // faz cópias da imagem para varredura em direções diferentes
	uint8_t top_bottom[TILE][TILE], bottom_up[TILE][TILE];
	uint8_t left_right[TILE][TILE], right_left[TILE][TILE];
   imgcopy_uint8(img, top_bottom);
   imgcopy_uint8(img, bottom_up);
   imgcopy_uint8(img, left_right);
   imgcopy_uint8(img, right_left);
   int strong = 255;
   // 1: de cima para baixo e esquerda para direita
   int i = 0, j = 0;
   for (i = 1; i < TILE; ++i)
       for (j = 1; j < TILE; ++j)
           if (top_bottom[i][j] == weak) {
               if (has_strong_neighbor(top_bottom, i, j)) {
                   top_bottom[i][j] = strong; // promove o pixel fraco a forte caso tenha um vizinho forte
               } else {
                   top_bottom[i][j] = 0; // caso contrario zera o pixel e isso se repete nas 4 varreduras
               }
           };
   // 2: de baixo para cima e direita para esquerda
   for (i = TILE - 2; i >= 1; --i)
       for (j = TILE - 2; j >= 1; --j)
           if (bottom_up[i][j] == weak) {
               if (has_strong_neighbor(bottom_up, i, j)) {
                   bottom_up[i][j] = strong;
               } else {
                   bottom_up[i][j] = 0;
               }
           };
   // 3: da esquerda para direita e de baixo para cima
   for (i = 1; i < TILE - 1; ++i)
       for (j = TILE - 2; j >= 1; --j)
           if (right_left[i][j] == weak) {
               if (has_strong_neighbor(right_left, i, j)) {
                   right_left[i][j] = strong;
               } else {
                   right_left[i][j] = 0;
               }
           };
   // 4: da direita para esquerda e de cima para baixo
   for (i = TILE - 2; i >= 1; --i)
       for (j = 1; j < TILE - 1; ++j)
           if (left_right[i][j] == weak) {
               if (has_strong_neighbor(left_right, i, j)) {
                   left_right[i][j] = strong;
               } else {
                   left_right[i][j] = 0;
               }
           }
   // qualquer pixel que foi posto em evidencia nos passos anteriores em pelo menos uma passagem
   // vira um pixel forte (vai pra 255)
   for (i = 0; i < TILE; ++i) {
       for (j = 0; j < TILE; ++j) {
           int soma = top_bottom[i][j] + bottom_up[i][j] + left_right[i][j] +
                      right_left[i][j];
           img[i][j] = (soma > 255) ? 255 : soma;
       }
   }
}

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
   HAL_Init();
   SystemClock_Config();
   MX_GPIO_Init();
   MX_USART2_UART_Init();
   HAL_Delay(300);
   // cabeçalho recebido via serial
   uint8_t header[8];
   uint8_t start;
   while (1)
       {
           // Espera byte de início 'S'
           HAL_UART_Receive(&huart2, &start, 1, HAL_MAX_DELAY);
           if (start != 'S') continue;
           // Recebe cabeçalho
           HAL_UART_Receive(&huart2, header, 8, HAL_MAX_DELAY);
           // Recebe tile
           for (int i = 0; i < TILE; i++)
               for (int j = 0; j < TILE; j++)
                   HAL_UART_Receive(&huart2, &tile[i][j], 1, HAL_MAX_DELAY);
           // Envia confirmação de início de eco
           HAL_UART_Transmit(&huart2, (uint8_t*)"S", 1, HAL_MAX_DELAY);
           // chamando as funcoes do canny
           filtro_gaussiano(tile);
           sobel_edge_detection(tile, grad_mag, grad_dir);
           non_max_suppression(grad_mag, grad_dir, tile);
           int weak = 75;
           threshold(tile, 5, 20, weak, tile);
           hysteresis(tile, weak);
           // Ecoa cabeçalho e tile
           HAL_UART_Transmit(&huart2, header, 8, HAL_MAX_DELAY);
           HAL_UART_Transmit(&huart2, &tile[0][0], TILE * TILE, HAL_MAX_DELAY);
           // Envia fim
           HAL_UART_Transmit(&huart2, (uint8_t*)"E", 1, HAL_MAX_DELAY);
       }
}
// === Clock e UART ===
void SystemClock_Config(void)
{
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
   RCC_OscInitStruct.HSICalibrationValue = 16;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
   HAL_RCC_OscConfig(&RCC_OscInitStruct);
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
   HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}
static void MX_GPIO_Init(void)
{
   __HAL_RCC_GPIOA_CLK_ENABLE();
}
static void MX_USART2_UART_Init(void)
{
   huart2.Instance = USART2;
   huart2.Init.BaudRate = 115200;
   huart2.Init.WordLength = UART_WORDLENGTH_8B;
   huart2.Init.StopBits = UART_STOPBITS_1;
   huart2.Init.Parity = UART_PARITY_NONE;
   huart2.Init.Mode = UART_MODE_TX_RX;
   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart2.Init.OverSampling = UART_OVERSAMPLING_16;
   HAL_UART_Init(&huart2);
}
