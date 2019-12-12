/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Softstarter
  * @author			: Cássio F. Braga e Nathan E. Nogara
  * @version		: 1.12
  * @date			: 12/09/2019
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//Inclui biblioteca de configuração do I2C do display LCD
#include "i2c-lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Ângulo mínimo e máximo para disparo (em graus)
#define ANG_MIN 49
#define ANG_MAX 349

//Largura do pulso do disparo
#define LARGURA_PULSO 10

//Conversão do ADC
#define CONV_FL 3300/1024
#define CONV_FL_V 282/1024

//Número de conversões do ADC
#define NDMA 2

//Tamanho dos caracteres da linha do display
#define MTRX_SIZE 20
#define MTRX_SIZE_USART 36
#define MTRX_SIZE_USART_RX 36

//Cálculo do timer do disparo
#define PSC_CLOCK 0.00011111111111           //0.00010131712259       //0.00009999
#define CICLO PSC_CLOCK*(ANG_MAX-ANG_MIN)

//Valor de meio segundo
#define MEIO_SEC 15

//Multiplicador máximo do tempo de ativação e desativação
#define MAX_SEC 110
#define MIN_SEC 10

//Para a sobrecorrente
#define AUM_CUR 50
#define MAX_CUR 1000
#define MULT_CUR 0.8

//Nomes predefinidos
#define ATIVANDO "Acelerando    *     "
#define DESATIVANDO "Desacelerando *     "
#define DESATIVADO "Desativado    *     "
#define REGIME "Regime        *     "
#define EMERGENCIA "Emergencia    *     "
#define VAZIO "                    "

//Envia o estado atual para o bluetooth
#define ATIVANDO_BT "EA"
#define DESATIVANDO_BT "ED"
#define DESATIVADO_BT "ES"
#define REGIME_BT "ER"
#define EMERGENCIA_BT "EE"

//Verifica as variáveis recebidas do bluetooth
#define BT_A 'T'
#define BT_D 'E'
#define BT_E 'M'
#define BT_S 'S'

//Flag
#define ON 1
#define OFF 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//Variáveis do tipo Double
double corrente_val, time=0, t_sobe, t_desce, tensao_val;

//Variáveis de 8 bits
uint8_t sinal_led=0; //Sinaliza o estado através do Led RGB
uint8_t button[5] = {0, 0, 0, 0, 0}, bot[5] = {0, 0, 0, 0, 0}, botCel[5] = {0, 0, 0, 0, 0}; //Buffers dos botões e botões do celular
uint8_t usart[MTRX_SIZE_USART] = {0}, usart_rx[MTRX_SIZE_USART] = {0}; //Buffer de transmissão da usart
uint8_t *corrente, *estado, *estado_BT, t_subida[MTRX_SIZE], *tempo, estado_atual[MTRX_SIZE], display_tempo[MTRX_SIZE], disp_val[MTRX_SIZE], *disp_temp; //Ponteiros e strings do que é enviado para a IHM
uint8_t pulsedown = 0, rampa_sobe = 0, rampa_desce = 0, ativa = 0, sd = 0, emergencia = 0, alimenta = 0, atualiza = 1; //Sinalizações de rampas, emergência, atualização, IHM e pulso
char buf_s[2], buf_d[2], buf_c[4]; //Buffer dos valores de subida, descida e corrente do celular

//Variáveis do tipo int
int conv_s, conv_d, conv_c; //Recebe os valores dos buffers de subida, descida e corrente convertidos para inteiros

//Variáveis de 16 bits
uint16_t pulso[2], angulo = 349, mult_s = MIN_SEC*2, mult_d = MIN_SEC*2; //angulo varia de 49 a 349
uint16_t arr_s = MEIO_SEC*MIN_SEC*2, arr_d = MEIO_SEC*MIN_SEC*2; //Valor para variar o tempo
uint16_t cur = MAX_CUR; //Valor de sobre corrente (na rampa de subida, não deve ultrapassar 80% desse valor)

//Variáveis do AD
uint16_t dado[NDMA]; //Dados do ADC

//Configuração do pino do disparo da soft
GPIO_InitTypeDef GPIO_InitStructA = {0}, GPIO_InitStructB = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void DisplayMain(void);
//void Emergencia(void);	//Antigo
void Led(uint8_t Val);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  //Pino PA6 (disparo da soft)
  GPIO_InitStructA.Pin = GPIO_PIN_6;
  GPIO_InitStructA.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructA.Pull = GPIO_NOPULL;
  GPIO_InitStructA.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructA.Alternate = GPIO_AF2_TIM3;

  GPIO_InitStructB.Pin = GPIO_PIN_6;
  GPIO_InitStructB.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructB.Pull = GPIO_NOPULL;
  GPIO_InitStructB.Speed = GPIO_SPEED_FREQ_LOW;

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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  //Iniciando LCD
  lcd_init ();

  //Inicializando Timers
  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim14);

  //Inicializando ADC
  HAL_ADC_Start_DMA(&hadc1, dado, NDMA);

  //Setando ponteiros de variáveis
  estado = &estado_atual;
  corrente = &disp_val;
  tempo = &t_subida;
  disp_temp = &display_tempo;

  //Desativa saída da soft inicialmente
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
  //Ativa o pino de disparo como saída normal inicialmente
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructB);

  //Inicializando USART6
  HAL_UART_Transmit_DMA(&huart6, usart, MTRX_SIZE_USART);
  HAL_UART_Receive_DMA(&huart6, &usart_rx, MTRX_SIZE_USART_RX);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	DisplayMain();
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//! @brief Programa proximo evento
//! @param handler do timer que gerou a interrupcao
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (pulsedown==0) // se pulso estiver desligado
	{
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,pulso[1]);
		pulsedown=1;
	}
	else
	{
		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,pulso[0]);
		pulsedown=0;
	}
}

//! @brief Interrupção dos timers
//! @param handler do timer que gerou a interrupcao
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//Timer que controla o ângulo de disparo
	if(htim -> Instance == TIM7) //Se for o timer 7 que recebe a interrupção, eu faço...
	{
		if(rampa_sobe == 1 && !emergencia)
		{
			time = PSC_CLOCK*arr_s*(angulo-ANG_MIN);
			if(angulo > ANG_MIN)
			{
				if(corrente_val>cur*MULT_CUR)
				{
					angulo++;
				}
				else
				{
					angulo--;
				}
				sprintf(tempo, "Ta=%5.2fs Cm=%4dmA", time, cur);
			}
			else
			{
				rampa_sobe = 0;
				if(!emergencia)
				{
					HAL_GPIO_TogglePin(Relay_GPIO_Port, Relay_Pin);
				}
				//HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
				HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6); //Desativa o pino de disparo
				//Ativa o pino de disparo
				HAL_GPIO_Init(GPIOA, &GPIO_InitStructB);
			}
		}
		else if(rampa_desce == 1 && !emergencia)
		{
			time = PSC_CLOCK*arr_d*(ANG_MAX-angulo);
			if(angulo < ANG_MAX)
			{
				angulo++;
				sprintf(tempo, "Td=%5.2fs Cm=%4dmA", time, cur);
			}
			else
			{
				rampa_desce = 0;
				//Desativa o pino de disparo
				HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
				//Ativa o pino de disparo em saída normal
				HAL_GPIO_Init(GPIOA, &GPIO_InitStructB);
			}
		}
		else
		{
			time = 0;
			if(ativa)
			{
				if(!emergencia)
				{
					//Desativa o pino de disparo
					HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
					//Ativa o pino de disparo em saída do timer
					HAL_GPIO_Init(GPIOA, &GPIO_InitStructA);
				}

				if(emergencia)
				{
					__HAL_TIM_SET_AUTORELOAD(&htim7, arr_d);
					if(estado == REGIME)
					{
						HAL_GPIO_TogglePin(Relay_GPIO_Port, Relay_Pin);
					}
					angulo = ANG_MAX;
					rampa_sobe = 0;
					rampa_desce = 1;
				}
				else if(angulo == ANG_MAX)
				{
					__HAL_TIM_SET_AUTORELOAD(&htim7, arr_s);
					rampa_sobe = 1;
					estado = ATIVANDO;
					estado_BT = ATIVANDO_BT;
					sinal_led = 1;
				}
				else
				{
					__HAL_TIM_SET_AUTORELOAD(&htim7, arr_d);
					rampa_desce = 1;
					estado = DESATIVANDO;
					estado_BT = DESATIVANDO_BT;
					sinal_led = 3;
					HAL_GPIO_TogglePin(Relay_GPIO_Port, Relay_Pin);
				}
				ativa = 0;
			}
			else
			{

				switch(sd)
				{
					case 0:
						sprintf(tempo, "T = 0s   S          ");
					break;

					case 1:
						sprintf(tempo, "T = 0s   D          ");
					break;

					case 2:
						sprintf(tempo, "T = 0s   Cm=%4dmA  ", cur);
					break;

					case 3:
						sprintf(tempo, "T = 0s   V          ");
					break;

				}

				if(angulo == ANG_MIN)
				{
					estado = REGIME;
					estado_BT = REGIME_BT;
					sinal_led = 2;
				}
				else
				{
					if(emergencia)
					{
						estado = EMERGENCIA;
						estado_BT = EMERGENCIA_BT;
						sinal_led = 4;
					}
					else
					{
						estado = DESATIVADO;
						estado_BT = DESATIVADO_BT;
						sinal_led = 0;
					}
				}
			}
		}
	}

	pulso[1] = LARGURA_PULSO + (pulso[0] = angulo);

	//Tempo de subida e descida da rampa no display
	t_sobe = CICLO*arr_s;
	t_desce = CICLO*arr_d;

	sprintf(disp_temp, "Ts=%5.2fs, Td=%5.2fs", t_sobe, t_desce);

	//Timer de 0,5 segundos para atualização da IHM e emergência
	if(htim -> Instance == TIM6) //Se for o timer 6 que recebe a interrupção, eu faço...
	{
		if(emergencia > 0 && emergencia < 20) //Emergencia por 10 segundos
		{
			emergencia++;

			if(alimenta)
			{
				HAL_GPIO_TogglePin(Rede_GPIO_Port, Rede_Pin);
				alimenta = 0;
			}

			if(HAL_GPIO_ReadPin(Relay_GPIO_Port, Relay_Pin))
			{
				HAL_GPIO_TogglePin(Relay_GPIO_Port, Relay_Pin);
			}

			//Desativa o pino de disparo
			HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
			//Ativa o pino de disparo como saída do timer normal
			HAL_GPIO_Init(GPIOA, &GPIO_InitStructB);
		}
		else
		{
			emergencia = 0;
		}

		//display atualiza de 0,5 em 0,5 segundos
		atualiza=1;
	}
	//Controle dos botões da IHM
	if(htim -> Instance == TIM14) //Se for o timer 14 que recebe a interrupção, eu faço...
	{
		//bot é a flag de botão já pressionado
		//Faz as verificações para ativar, incrementar, decrementar ou ativar a emergência de acordo com o botão pressionado
		if(!HAL_GPIO_ReadPin(VaiSoft_GPIO_Port, VaiSoft_Pin) && button[0] == 1 && bot[0] == 0)
			{
				bot[0] = 1;
				if(alimenta == 0 && !emergencia)
				{
					HAL_GPIO_TogglePin(Rede_GPIO_Port, Rede_Pin);
					alimenta = 1;
				}
				else if(rampa_sobe || rampa_desce)
				{
					ativa = 0;
				}
				else if(sd == 3 && !emergencia)
				{
					sd = 0;
					ativa = 1;
				}
			}
		else if(!HAL_GPIO_ReadPin(AumentaTempo_GPIO_Port, AumentaTempo_Pin) && button[1] == 1 && bot[1] == 0)
			{
				bot[1] = 1;
				if(!(rampa_sobe || rampa_desce) && !emergencia)
				{
					if(sd == 1)
					{
						if(mult_d < MAX_SEC)
						{
							mult_d+=2;
							arr_d = MEIO_SEC*mult_d;
						}
					}
					else if(sd == 0)
					{
						if(mult_s < MAX_SEC)
						{
							mult_s+=2;
							arr_s = MEIO_SEC*mult_s;
						}
					}
					else if(sd == 2)
					{
						if(cur < MAX_CUR)
						{
							cur += AUM_CUR;
						}
					}
				}
			}
		else if(!HAL_GPIO_ReadPin(DiminuiTempo_GPIO_Port, DiminuiTempo_Pin) && button[2] == 1 && bot[2] == 0)
			{
				bot[2] = 1;
				if(!(rampa_sobe || rampa_desce) && !emergencia)
				{
					if(sd == 1)
					{
						if(mult_d > MIN_SEC)
						{
							mult_d-=2;
							arr_d = MEIO_SEC*mult_d;
						}
					}
					else if(sd == 0)
					{
						if(mult_s > MIN_SEC)
						{
							mult_s-=2;
							arr_s = MEIO_SEC*mult_s;
						}
					}
					else if(sd == 2)
					{
						if(cur > AUM_CUR)
						{
							cur -= AUM_CUR;
						}
					}
				}
			}
		else if(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) && button[3] == 1 && bot[3] == 0)
			{
				bot[3] = 1;
				if(!(rampa_sobe || rampa_desce) && !emergencia)
				{
					if(sd < 3)
					{
						sd++;
					}
					else
					{
						sd = 0;
					}
				}
			}
		else if(!HAL_GPIO_ReadPin(Emerg_GPIO_Port, Emerg_Pin) && button[4] == 1 && bot[4] == 0)
			{
				bot[4] = 1;
				emergencia = 1;
				ativa = 1;
			}

		//Ativa flag do botão
		if(!HAL_GPIO_ReadPin(VaiSoft_GPIO_Port, VaiSoft_Pin) && button[0] == 0)
		{
			button[0] = 1;
		}
		else if(!HAL_GPIO_ReadPin(AumentaTempo_GPIO_Port, AumentaTempo_Pin) && button[1] == 0)
		{
			button[1] = 1;
		}
		else if(!HAL_GPIO_ReadPin(DiminuiTempo_GPIO_Port, DiminuiTempo_Pin) && button[2] == 0)
		{
			button[2] = 1;
		}
		else if(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) && button[3] == 0)
		{
			button[3] = 1;
		}
		else if(!HAL_GPIO_ReadPin(Emerg_GPIO_Port, Emerg_Pin) && button[4] == 0)
		{
			button[4] = 1;
		}

		//Resetam os valores dos botões
		if(HAL_GPIO_ReadPin(VaiSoft_GPIO_Port, VaiSoft_Pin))
		{
			button[0] = 0;
			bot[0] = 0;
		}

		if(HAL_GPIO_ReadPin(AumentaTempo_GPIO_Port, AumentaTempo_Pin))
		{
			button[1] = 0;
			bot[1] = 0;
		}

		if(HAL_GPIO_ReadPin(DiminuiTempo_GPIO_Port, DiminuiTempo_Pin))
		{
			button[2] = 0;
			bot[2] = 0;
		}

		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
		{
			button[3] = 0;
			bot[3] = 0;
		}

		if(HAL_GPIO_ReadPin(Emerg_GPIO_Port, Emerg_Pin))
		{
			button[4] = 0;
			bot[4] = 0;
		}
	}
}

//! @brief Fim da conversão AD de tensão e corrente
//! @param handler do AD
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	corrente_val = (float)CONV_FL*dado[0];
	tensao_val = (float)CONV_FL_V*dado[1];

	sprintf(corrente, "C=%4.0fmA  V=%3.0fV    ", corrente_val, tensao_val);

	if(corrente_val > cur)
	{
		emergencia = 1;
		ativa = 1;
	}

	HAL_ADC_Start_DMA(&hadc1, dado, NDMA);
}

//! @brief Fim do envio da USART
//! @param handler da USART
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_DMA(&huart6, &usart_rx, MTRX_SIZE_USART_RX);
}

//! @brief Fim do recebimento da USART
//! @param handler da USART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Recebe valores
	if(usart_rx[1] == BT_A)
	{
		if(estado == DESATIVADO)
		{
			if(alimenta == 0 && !emergencia)
			{
				HAL_GPIO_TogglePin(Rede_GPIO_Port, Rede_Pin);
				alimenta = 1;
			}
			else
			{
				sd = 0;
				ativa = 1;
			}
		}
	}
	else if(usart_rx[1] == BT_D)
	{
		if(estado == REGIME)
		{
			sd = 0;
			ativa = 1;
		}
	}
	else if(usart_rx[1] == BT_E)
	{
		emergencia = 1;
		ativa = 1;
	}
	else if(usart_rx[3] == BT_S)
	{
		int hm = 0, i=0, j=0, k=0, l=0;

		buf_s[0] = usart_rx[1];
		buf_s[1] = usart_rx[2];
		buf_d[0] = usart_rx[5];
		buf_d[1] = usart_rx[6];

		buf_c[0] = usart_rx[9];
		buf_c[1] = usart_rx[10];
		buf_c[2] = usart_rx[11];
		buf_c[3] = usart_rx[12];

		conv_s = atol(buf_s);
		conv_d = atol(buf_d);
		cur = conv_c = atol(buf_c);

		mult_s = conv_s*2;
		arr_s = MEIO_SEC*mult_s;

		mult_d = conv_d*2;
		arr_d = MEIO_SEC*mult_d;
	}

	//Envia para o bluetooth
	sprintf(usart, "&C%4.0fCV%3.0fVS%5.2fSD%5.2fDA%5.2fA%s$", corrente_val, tensao_val, t_sobe, t_desce, time,estado_BT);
	HAL_UART_Transmit_DMA(&huart6, &usart, MTRX_SIZE_USART);

}

//! @brief Cores do Led
//! @param Valor da cor de acordo com o manual
void Led(uint8_t Val)
{
	switch(Val)
	{
		//Led Branco (desativado)
		case 0:
			HAL_GPIO_WritePin(Vermelho_GPIO_Port, Vermelho_Pin, ON);
			HAL_GPIO_WritePin(Verde_GPIO_Port, Verde_Pin, ON);
			HAL_GPIO_WritePin(Azul_GPIO_Port, Azul_Pin, ON);
		break;

		//Led Amarelo (ativando)
		case 1:
			HAL_GPIO_WritePin(Vermelho_GPIO_Port, Vermelho_Pin, ON);
			HAL_GPIO_WritePin(Verde_GPIO_Port, Verde_Pin, ON);
			HAL_GPIO_WritePin(Azul_GPIO_Port, Azul_Pin, OFF);
		break;

		//Led Verde (regime)
		case 2:
			HAL_GPIO_WritePin(Vermelho_GPIO_Port, Vermelho_Pin, OFF);
			HAL_GPIO_WritePin(Verde_GPIO_Port, Verde_Pin, ON);
			HAL_GPIO_WritePin(Azul_GPIO_Port, Azul_Pin, OFF);
		break;

		//Led Azul (desativando)
		case 3:
			HAL_GPIO_WritePin(Vermelho_GPIO_Port, Vermelho_Pin, OFF);
			HAL_GPIO_WritePin(Verde_GPIO_Port, Verde_Pin, OFF);
			HAL_GPIO_WritePin(Azul_GPIO_Port, Azul_Pin, ON);
		break;

		//Led Vermelho (emergência)
		case 4:
			HAL_GPIO_WritePin(Vermelho_GPIO_Port, Vermelho_Pin, ON);
			HAL_GPIO_WritePin(Verde_GPIO_Port, Verde_Pin, OFF);
			HAL_GPIO_WritePin(Azul_GPIO_Port, Azul_Pin, OFF);
		break;
	}
}

//! @brief O que é amostrado no display LCD
//! @param Void
void DisplayMain(void)
{
	  if(atualiza)
	  {
		//Limpa tela
		lcd_send_cmd (0x80); // Linha 1
		lcd_send_string (VAZIO);
		lcd_send_cmd (0x94); // Linha 3
		lcd_send_string (VAZIO);
		lcd_send_cmd (0xC0); // Linha 2
		lcd_send_string (VAZIO);
		lcd_send_cmd (0xD4); // Linha 4
		lcd_send_string (VAZIO);

		//print no display
		lcd_send_cmd (0x80); // Linha 1
		lcd_send_string (disp_temp);
		lcd_send_cmd (0x94); // Linha 3
		lcd_send_string (estado);
		lcd_send_cmd (0xC0); // Linha 2
		lcd_send_string (tempo);
		lcd_send_cmd (0xD4); // Linha 4
		lcd_send_string (corrente);

		Led(sinal_led);

		//Envia para o bluetooth
		sprintf(usart, "&C%4.0fCV%3.0fVS%5.2fSD%5.2fDA%5.2fA%s$", corrente_val, tensao_val, t_sobe, t_desce, time,estado_BT);
		HAL_UART_Transmit_DMA(&huart6, &usart, MTRX_SIZE_USART);

		//Zera a flag de atualização da IHM
		atualiza = 0;
	  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
