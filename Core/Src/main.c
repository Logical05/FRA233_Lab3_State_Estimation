/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HCSR04.h"
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT 1e-3f
#define BOX_MASS 0.155f
#define NUT_MASS 0.038f

// Editable
#define NUT_NUM 1
#define K_SPRING 28.13852387f
#define C_DAMPING 0.001f

//#define X_EQ 9.706151498f // 5 Nut
//#define X_EQ 11.06710543f // 4 Nut
//#define X_EQ 12.78103434f // 3 Nut
//#define X_EQ 14.33077597f // 2 Nut
//#define X_EQ 15.78296892f   // 1 Nut

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TOTAL_MASS (BOX_MASS + (NUT_MASS * NUT_NUM))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
HCSR04_TypeDef hcsr = { 0 };
float distance = 0;
float kf_distance[4] = { 0 };
KF_TypeDef kf[4] = { 0 };

// Editable
float tune_q[4] = { 1e-3f, 1e-6f, 1e-9f, 0.048f };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void KF_Constant_Init(KF_TypeDef*, float, float);		// Velocity = 0
void KF_MSD_Init(KF_TypeDef*, float, float);	// Mass-Spring-Damper : u = 0
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	// HCSR04 (All Timer is MHz)
	hcsr.TRIG_PWM_Timer.htim = &htim3;
	hcsr.TRIG_PWM_Timer.TIM_Channal = TIM_CHANNEL_1;
	hcsr.ECHO_IC_Timer.htim = &htim1;
	hcsr.ECHO_IC_Timer.TIM_Channal = TIM_CHANNEL_1;
	hcsr.ECHO_IC_Timer.TIM_Active_Channal = HAL_TIM_ACTIVE_CHANNEL_1;
	HCSR04_Init(&hcsr);
	HCSR04_Start();

	// Kalman Filter
	HAL_TIM_Base_Start_IT(&htim4);	// 1000Hz
	KF_Constant_Init(&kf[0], tune_q[0], 0.010799226f);
	KF_Constant_Init(&kf[1], tune_q[1], 0.010799226f);
	KF_Constant_Init(&kf[2], tune_q[2], 0.010799226f);
//	KF_MSD_Init(&kf[3], tune_q[3], 0.010799226f);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	HCSR04_Echo_Callback(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim4) {
		kf[0].Q[0][0] = tune_q[0];
		kf[1].Q[0][0] = tune_q[1];
		kf[2].Q[0][0] = tune_q[2];
//		KF_SetQ_Discrete(&kf[3], tune_q[3], DT);

		KF_Predict(&kf[0]);
		KF_Predict(&kf[1]);
		KF_Predict(&kf[2]);
//		KF_Predict(&kf[3]);

		distance = HCSR04_Read();

		KF_Update(&kf[0], distance);
		KF_Update(&kf[1], distance);
		KF_Update(&kf[2], distance);
//		KF_Update(&kf[3], distance - X_EQ);

		kf_distance[0] = kf[0].x[0];
		kf_distance[1] = kf[1].x[0];
		kf_distance[2] = kf[2].x[0];
//		kf_distance[3] = kf[3].x[0] + X_EQ;
	}
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void KF_Constant_Init(KF_TypeDef *kf, float Q, float R) {
	KF_Init(kf);

	// Physics model
	kf->F[0][0] = 1.0f;
	kf->F[0][1] = 0.0f;
	kf->F[1][0] = 0.0f;
	kf->F[1][1] = 0.0f;

	// Process noise
	kf->Q[0][0] = Q;
	kf->Q[0][1] = 0.0f;
	kf->Q[1][0] = 0.0f;
	kf->Q[1][1] = 0.0f;

	// Measurement noise
	kf->R = R;
}

void KF_MSD_Init(KF_TypeDef *kf, float sigma_a, float R) {
	KF_Init(kf);

	// Physics model
	float k_m = K_SPRING / TOTAL_MASS;
	float c_m = C_DAMPING / TOTAL_MASS;

	kf->F[0][0] = 1.0f;
	kf->F[0][1] = DT;
	kf->F[1][0] = -k_m * DT;
	kf->F[1][1] = 1.0f - (c_m * DT);

	// Process noise
	KF_SetQ_Discrete(kf, sigma_a, DT);

	// Measurement noise
	kf->R = R;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
