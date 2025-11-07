// algoritmo stm32cube

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PI 3.14159265
#define ALTURA 89
#define LARGURA 89

uint8_t img[89][89] = {0}; //< Matriz que recebe a imagem de entrada
//float mag[89][89]; //< Matriz para armazenar a magnitude do gradiente
//float dir[89][89]; //< Matriz para armazenar a direção do gradiente
//uint8_t output[55][55]; //< Matriz que receberá a imagem de saída

int largura = LARGURA, altura = ALTURA; // atribuindo o valor das constantes às variáveis largura e altura

void imgcopy(uint8_t input[ALTURA][LARGURA], uint8_t output[ALTURA][LARGURA]) {
    for(int i = 0; i < ALTURA; ++i)
        for(int j = 0; j < LARGURA; ++j)
            output[i][j] = input[i][j];
}

/**
 * @brief   aplica o Gaussian Blur para redução de ruídos na imagem
 */
void filtro_gaussiano() {
    uint8_t temp[89][89]; //< buffer temporário

    // kernel padrão 5x5 pré normalizado com a soma dos elementos da matriz
    float kernel[5][5] = {
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
void convolution(uint8_t image[ALTURA][LARGURA], int kernel[3][3], uint8_t output[ALTURA][LARGURA]) {
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
void sobel_edge_detection(uint8_t image[ALTURA][LARGURA]/*, float grad_mag[ALTURA][LARGURA], float grad_dir[ALTURA][LARGURA]*/) {
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
    float grad_mag[89][89], grad_dir[89][89];

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

    non_max_suppression(grad_mag, grad_dir, img);
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
void hysteresis(uint8_t img[ALTURA][LARGURA], int weak) {
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    filtro_gaussiano();

    sobel_edge_detection(img);

    //non_max_suppression(mag, dir, img);

    int weak = 75;
    threshold(img, 5, 20, weak, img);

    hysteresis(img, weak);
    //HAL_UART_Transmit(&huart2, output, 10, 1000);


    //salvar_pgm("output.pgm", output);
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        // HAL_UART_Transmit(&huart2, output, 10, 1000);
        HAL_Delay(10000);
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
