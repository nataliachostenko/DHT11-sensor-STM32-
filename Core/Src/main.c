/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_FRAME_SIZE 1024
#define DHT11_PORT DHT11_GPIO_Port
#define DHT11_PIN DHT11_Pin

#define RX_BF_length 1024
#define TX_BF_length 2048
#define BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;
volatile uint8_t tempInterval = 20;
volatile uint8_t humInterval = 20;


uint8_t RX_BF[RX_BF_length];
uint8_t TX_BF[TX_BF_length];

volatile int RX_IDX_EMPTY = 0;
volatile int RX_IDX_BUSY = 0;
volatile int TX_IDX_EMPTY = 0;
volatile int TX_IDX_BUSY = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void FrameRd();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//kod
typedef struct {
	char data[BUFFER_SIZE];
	int start;
	int end;
} circular_buffer_t;

void cb_init(circular_buffer_t *cb) {
  cb->start = 0;
  cb->end = 0;
  memset(cb->data, '\0', sizeof(char)*BUFFER_SIZE);
}

void cb_push_back(circular_buffer_t *cb, char item) {
  cb->data[cb->end] = item;
  cb->end = (cb->end + 1) % BUFFER_SIZE;
  if (cb->end == cb->start) {
    cb->start = (cb->start + 1) % BUFFER_SIZE;
  }
}

char cb_get_first(circular_buffer_t *cb) {
  if (cb->start == cb->end) {
    return '\0';
  }
  return cb->data[cb->start];
}

char cb_get_last(circular_buffer_t *cb) {
  if (cb->start == cb->end) {
    return '\0';
  }
  int index = cb->end == 0 ? BUFFER_SIZE - 1 : cb->end - 1;
  return cb->data[index];
}

int cb_size(circular_buffer_t *cb) {
    if(cb->end >= cb->start){
        return cb->end - cb->start;
    }
    else {
        return BUFFER_SIZE - (cb->start - cb->end);
    }
}

int cb_get_start_index(circular_buffer_t *cb)
{
	return cb->end;
}

int cb_get_end_index(circular_buffer_t *cb)
{
	return cb->end;
}

char cb_get_nth_last(circular_buffer_t *cb, int n) {
  int index = (cb->end - n - 1 + BUFFER_SIZE) % BUFFER_SIZE;
  if (index < 0 || index >= BUFFER_SIZE || cb->start == cb->end) {
	  return '\0';
  }
  return cb->data[index];
}

char cb_get_nth_first(circular_buffer_t *cb, int n) {
  int index = (cb->start + n) % BUFFER_SIZE;
  if (index < 0 || index >= BUFFER_SIZE || cb->start == cb->end) {
	  return '\0';
  }
  return cb->data[index];
}


circular_buffer_t temp;
circular_buffer_t rh;


void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}



volatile DHT11_State dht11_state = DHT11_IDLE;
volatile uint32_t msTicks = 0; // Licznik ticków SysTick, każdy tick = 1ms
uint8_t data[5] = {0}; // Bufor na otrzymane dane
uint8_t bits[40] = {0}; // Bufor na 40 otrzymanych bitów
volatile int bitIndex = 0; // Indeks do śledzenia odbieranych bitów

// Inicjalizacja SysTicka
void SysTick_Init(void) {
    SysTick_Config(SystemCoreClock / 1000); // Konfiguracja przerwań co 1 ms
}

void DHT11_Start(void) {
    Set_Pin_Output(DHT11_PORT, DHT11_PIN);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET); // Niski poziom
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
    dht11_state = DHT11_START;
    msTicks = 0; // rozpoczecie odliczania
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    static uint32_t lastCaptureValue = 0;
    static uint8_t bitsIndex = 0; // Resetowany, gdy rozpoczety odbiór danych
    static uint8_t data[5] = {0}; // Bufor na dane
    uint32_t pulseLength;
//wystartowac timer w input capture
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        uint32_t captureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Odczyt wartości przechwyconej

        // Zapobieganie przewinięciu licznika
        if (captureValue > lastCaptureValue) {
            pulseLength = captureValue - lastCaptureValue;
        } else {
            // Uwzględnienie przewinięcia licznika timera
            pulseLength = (0xFFFF - lastCaptureValue + captureValue + 1);
        }

        lastCaptureValue = captureValue;

        switch (dht11_state) {
            case DHT11_WAIT_FOR_RESPONSE:
                if (pulseLength >= 20 && pulseLength <= 40) {
                    dht11_state = DHT11_RESPONSE_LOW;
                } else {
                    dht11_state = DHT11_ERROR;
                }
                break;
            case DHT11_RESPONSE_LOW:
                if (pulseLength >= 80) {
                    dht11_state = DHT11_RESPONSE_HIGH;
                }
                break;
            case DHT11_RESPONSE_HIGH:
                if (pulseLength >= 80) {
                    dht11_state = DHT11_RECEIVE_DATA;
                }
                break;
            case DHT11_RECEIVE_DATA:
                if (pulseLength > 50) { // Prog dla 1
                    data[bitsIndex / 8] <<= 1;
                    data[bitsIndex / 8] |= 1; // Dodanie 1 do bufora danych
                } else {
                    // Dla 0 nie trzeba nic robić, bit jest już 0 po przesunięciu bitowym w lewo
                    data[bitsIndex / 8] <<= 1;
                }
                bitsIndex++;
                if (bitsIndex >= 40) { // bity odebrane
                    dht11_state = DHT11_DATA_READY;
                    Process_Data(data);
                    bitsIndex = 0; // Resetowanie indeksu bitów
                    dht11_state = DHT11_IDLE;
                }
                break;
            case DHT11_ERROR:
                // Resetowanie na potrzeby kolejnego odczytu
                bitsIndex = 0;
                memset(data, 0, sizeof(data));
                msTicks = 0;
                dht11_state = DHT11_IDLE;
                break;
        }
    }
}

void Process_Data(uint8_t *data) {
    // Przypisanie odebranych danych do zmiennych
    Rh_byte1 = data[0];
    Rh_byte2 = data[1];
    Temp_byte1 = data[2];
    Temp_byte2 = data[3];
    SUM = data[4];

    // Obliczenie  RH i TEMP na podstawie otrzymanych danych
    RH = Rh_byte1;
    TEMP = Temp_byte1;

    // Sprawdzenie sumy kontrolnej
    uint16_t calculated_sum = Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2;
    if (calculated_sum == SUM) {
        // Dodanie wartości do buforów cyklicznych
        cb_push_back(&temp, TEMP);
        cb_push_back(&rh, RH);
    } else {
        dht11_state = DHT11_ERROR;
    }
}

//czy w UART są znaki do odczytu
uint8_t USART_kbhit()
{
	if (RX_IDX_EMPTY == RX_IDX_BUSY) return 0; //nie ma danych
	else return 1;
}

//odczyt znaku z bufora
int16_t USART_getChar()
{
	//tymczasowe miejsce na pobrany znak
	int16_t charTMP;
	//są znaki do odczytu
	if (RX_IDX_EMPTY != RX_IDX_BUSY)
	{
		//czytanie znaku
		charTMP = RX_BF[RX_IDX_BUSY];
		RX_IDX_BUSY++;
		if (RX_IDX_BUSY >= RX_BF_length) RX_IDX_BUSY = 0;
		return charTMP;
	}
	else return -1;
}

//odbiór z UART i zapis do bufora
uint8_t getLine(char *buf)
{
	// tymczasowe przechowywanie odebranych danych
	static uint8_t buffer[128];
	//indeks do pozycji w tymczasowym buforze
	static uint8_t idx = 0;
	int temp; //z temp do docelowego
	uint8_t ret;
	while(USART_kbhit())
	{
		buffer[idx]=USART_getChar();
		if (((buffer[idx] == 10)||(buffer[idx] == 13))) //czy \r lub \n
		{
			buffer[idx] = 0; //koniec stringa
			for(temp = 0; temp <= idx; temp++)
				{
					buf[temp] = buffer[temp];
				}
			ret = idx;
			idx = 0;
			//ilosc znakow w jednej linii
			return ret;
		}
		else
		{
			idx++;
			if (idx >= 128) idx = 0;
		}
		return 0;
	}
}

//wysyłanie danych przez UART
void USART_fsend(char* format, ...)
{
	//do sformatowanego ciagu
	char tmp_rs[128];
	int i;
	//zmienna do śledzenia pozycji w buforze nadawczym
	__IO int idx;
	va_list arglist;
	va_start(arglist,format);
	//zapis do bufora sformatowanego stringa
	vsprintf(tmp_rs,format,arglist);
	va_end(arglist);
	idx = TX_IDX_EMPTY;

	for(i = 0; i < strlen(tmp_rs) ; i++)
	{
		TX_BF[idx] = tmp_rs[i];
		idx++;
		if (idx >= TX_BF_length) idx = 0;
	}
	__disable_irq();

	//trzeba rozpoczac transmisje
	if ((TX_IDX_EMPTY == TX_IDX_BUSY) && (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET))
	{
		TX_IDX_EMPTY = idx;
		uint8_t temp = TX_BF[TX_IDX_BUSY];
		TX_IDX_BUSY++;
		if (TX_IDX_BUSY >= TX_BF_length) TX_IDX_BUSY = 0;
		//transmisja jednego znaku
		HAL_UART_Transmit_IT(&huart2, &temp, 1);
	} else TX_IDX_EMPTY = idx;
	__enable_irq();
}

//transmisja pomyślna
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//if (huart->Instance == 'huart2')
	//{
		if (TX_IDX_EMPTY != TX_IDX_BUSY)
		{
			//wysylanie kolejnego znaku
			uint8_t temp = TX_BF[TX_IDX_BUSY];
			TX_IDX_BUSY++;
			if (TX_IDX_BUSY >= TX_BF_length) TX_IDX_BUSY = 0;
			HAL_UART_Transmit_IT(&huart2, &temp, 1);
		}
	//}
}

//odebranie pomyślne
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//if (huart->Instance == 'huart2')
	//{
	//indeks gdzie mozna umiescic znak
		RX_IDX_EMPTY++;
		if (RX_IDX_EMPTY >= RX_BF_length)
		{
			RX_IDX_EMPTY = 0;
		}
		HAL_UART_Receive_IT(&huart2, &RX_BF[RX_IDX_EMPTY], 1);
	//}
}

typedef enum
{
	Waiting,
	Launch,
	FrameError
} FrameDetection;


int crc_checksum(char *arr, int start, int finish) {
  int polynomial = 0x07;

  int suma = 0;
  int r;

  for (int i = start; i < finish; i++) {
    r = suma ^ (arr[i] & 0xFF); //xor
    for (int j = 0; j < 8; j++) {
      if (r & 1) {
        r = (r >> 1) ^ polynomial;//xor
      } else {
        r >>= 1;
      }
    }
    suma = (suma >> 8) ^ r;//xor
  }

  return suma & 0xFF;
}


void sendOk() {
	USART_fsend("\r\n/OKK[]140;\r\n");
}

void send(char *text) {
  USART_fsend("\r\n/%s%03d;\r\n", text, crc_checksum(text, 0, strlen(text)));
}

void ProcessFrame(char *buffer, uint16_t buf_size);

void FrameRd() {
    static FrameDetection detection = Waiting;
    static char buffer[MAX_FRAME_SIZE] = {0};
	static char uncodedBuffer[MAX_FRAME_SIZE] = {0};
    static char tempBuffer[MAX_FRAME_SIZE] = {0}; // Bufor tymczasowy do zbierania nieprzekodowanych danych
    static uint16_t index_temp_buffer = 0;
    static uint16_t index_buffer_start = 0;
    uint16_t uncodedBufferSize = 0;

    int16_t sign;
    if (USART_kbhit()) {
        sign = USART_getChar();
    } else {
        return; //nie ma niczego od odczytu
    }

    if (sign < 0) return;

    tempBuffer[index_temp_buffer] = sign;
    index_temp_buffer++;

        if (sign == ';') 		//spr czy przyszedł znak konca ramki
        {

        		//				////////////ABCD.|E;;;;;;;;;;;;; 	<- tempBuffer
        		//						   /ABCD.|E;				<- uncodedBuffer
        	for(uint16_t i = 0; i < index_temp_buffer; i++)			//szukamy od konca pierwszego wystapienia znaku poczatku ramki
        	{
        		if(tempBuffer[index_temp_buffer-1-i] == '/')
        		{
        			index_buffer_start = index_temp_buffer-1-i;												//przypisanie pozycji poczatkowej do faktycznie poczatku ramki w sytuacji gdy wystapi wiele znakow
        			uncodedBufferSize = index_temp_buffer - i;
        			memcpy(uncodedBuffer, &tempBuffer[index_buffer_start], uncodedBufferSize);
        			detection = Launch;
        			break;																					//konczymy iterowanie w petli, nie ma koniecznosci szukania kolejnych znakow poczatku ramki (sa nieistotne)
        		}
        	}
        	//  			/ABCD.|E;   <- uncodedBuffer
        	//				/ABCD/E;	<- buffer
        	buffer[0] = uncodedBuffer[0]; 										//przepisanie znaku poczatku ramki '/'

        	uint16_t i;
        	for(i = 1; i < uncodedBufferSize - 1; i++)
        	{
        		if(uncodedBuffer[i] == '.' && uncodedBuffer[i+1] == ':')
        		{
        			buffer[i] = ';';
        			i++;												//pomin drugi znak kodujacy
        			continue;
        		}
        		if(uncodedBuffer[i] == '.' && uncodedBuffer[i+1] == '|')
				{
					buffer[i] = '/';
					i++;												//pomin drugi znak kodujacy
					continue;
				}
        		if(uncodedBuffer[i] == '.' && uncodedBuffer[i+1] == '.')
				{
					buffer[i] = '.';
					i++;												//pomin drugi znak kodujacy
					continue;
				}
        		if(uncodedBuffer[i] == '.' && uncodedBuffer[i+1] != '.')
				{
					index_temp_buffer = 0;
					return;
				}

        		buffer[i] = uncodedBuffer[i];							//przepisywanie pozostalych znakow

        	}

        	buffer[i] = uncodedBuffer[uncodedBufferSize - 1];		//przepisanie znaku konca ramki ';'

            if (detection == Launch) {
                detection = Waiting;
                ProcessFrame(buffer, i);								//przetworzenie danych

            }
        }


    // Sprawdzenie przekroczenia maksymalnego rozmiaru ramki
    if (index_temp_buffer >= MAX_FRAME_SIZE) {
        detection = FrameError;
        index_temp_buffer = 0;
        memset(buffer, 0, MAX_FRAME_SIZE);
        memset(tempBuffer, 0, MAX_FRAME_SIZE);
        send("INC[Frame Error]");
    }
}



void parseNumericArguments(const char *arguments, int *arg1, int *arg2) {
    char arg1Str[BUFFER_SIZE], arg2Str[BUFFER_SIZE];
    if (sscanf(arguments, "%[^,], %[^\t\n]", arg1Str, arg2Str) >= 1) {
        *arg1 = atoi(arg1Str);
    }
    if (arg2 != NULL && sscanf(arguments, "%[^,], %[^\t\n]", arg1Str, arg2Str) == 2) {
        *arg2 = atoi(arg2Str);
    }
}

// funkcja do porównywania stringów z wildcardem na inty
int compareStringWithIntWildcard(const char *pattern, const char *str) {
	//petla dziala dopoki nie napotka konca stringow
    while (*pattern != '\0' && *str != '\0') {
    	if (*pattern == '%') { // założenie że % to wildcard
			if (*pattern != 'd') { // szukanie d po %
				return 0; // niepoprawny wzór
			}
			pattern++;
			// przejście przez ciag cyfr w drugim stringu
			while (isdigit((unsigned char)*str)) {
				str++;
			}
		} else { //zwykle znaki z wzorca
			if (*pattern != *str) {
				return 0; // znaki się nie zgadzają
			}
			pattern++;
			str++;
		}
    }

// sytuacja, w której jeden string się kończy, a drugi wciąż posiada znaki
	if (*pattern == '\0' && *str == '\0') {
		return 1; // oba stringi kończą się w tym samym momencie, są zgodne
	} else {
		return 0; // jeden string się skończył, drugi nie, stringi nie są zgodne
	}
}



void ProcessFrame(char *buffer, uint16_t buf_size) {
    char frameData[MAX_FRAME_SIZE];
    //int frameSize = cb_size(frameBuffer);
    for (int i = 0; i < buf_size; i++) {
        frameData[i] = buffer[buf_size - i -1]; //cb_get_nth_last(frameBuffer, frameSize - i - 1); //kopiowanie danych w odpowiedniej kolejnosci
    }
    frameData[buf_size] = '\0';
    // /TMP[GET,1]879;
    // Rozkodowanie ramki
    char command[BUFFER_SIZE], arguments[BUFFER_SIZE];
    sscanf(frameData, "/%s %[^\t\n]", command, arguments);
    //command - TMP
    //arguments - GET,1
    // Inicjalizacja zmiennych liczbowych na domyślne wartości
    int arg1 = 0;
    int received_crc, calculated_crc;
    // Sprawdzenie sumy kontrolnej
	calculated_crc = crc_checksum(frameData, 0, strlen(frameData));
	sscanf(frameData, "%*[^]]]%d", &received_crc); // Odczyt sumy CRC z ramki
	if (calculated_crc != received_crc) {
		send("INC[INCORRECT CHECKSUM]");
	}

    if (strlen(arguments) > 0) {

        // Rozpoznawanie komendy i wykonanie odpowiednich akcji
        if (strcmp(command, "/TMP") == 0) {
            if (strcmp(arguments, "[GET]") == 0) {
                char s[BUFFER_SIZE];
                sprintf(s, "OKK[Temp now is %d]", cb_get_last(&temp));
                send(s);
            } else if (compareStringWithIntWildcard(arguments, "[GET,%d]") == 1) {
                char s[BUFFER_SIZE];
                parseNumericArguments(arguments, &arg1, NULL);
                sprintf(s, "OKK[Temp from -%d is %d]", arg1, cb_get_nth_last(&temp, arg1));
                send(s);
            } else if (compareStringWithIntWildcard(arguments, "[GET,%d,%d]") == 1) {
                int startIndex = 0, endIndex = 0;
                parseNumericArguments(arguments, &startIndex, &endIndex);
                char s[BUFFER_SIZE];
                sprintf(s, "OKK[Temp from -%d to -%d is", startIndex, endIndex);
                for (int i = startIndex; i <= endIndex; i++) {
                    int temperature = cb_get_nth_first(&temp, i);
                    sprintf(s + strlen(s), " %d", temperature);
                }
                sprintf(s + strlen(s), "]");
                send(s);
            } else if (cb_size(&temp) == 0) { // Sprawdzenie, czy bufor temp jest pusty
                send("EMP");
            }
        } else if (strcmp(command, "/HUM") == 0) {
            if (strcmp(arguments, "[GET]") == 0) {
                char s[BUFFER_SIZE];
                sprintf(s, "OKK[Humidity now is %d]", cb_get_last(&rh));
                send(s);
            } else if (compareStringWithIntWildcard(arguments, "[GET,%d]") == 0) {
                char s[BUFFER_SIZE];
                parseNumericArguments(arguments, &arg1, NULL);
                sprintf(s, "OKK[Humidity from -%d is %d]", arg1, cb_get_nth_last(&rh, arg1));
                send(s);
            } else if (compareStringWithIntWildcard(arguments, "[GET,%d,%d]") == 0) {
                int startIndex= 0, endIndex = 0;
                parseNumericArguments(arguments, &startIndex, &endIndex);
                char s[BUFFER_SIZE];
                sprintf(s, "OKK[Humidity from -%d to -%d is", startIndex, endIndex);
                for (int i = startIndex; i <= endIndex; i++) {
                	int humidityValue = cb_get_nth_first(&rh, i);
                	sprintf(s + strlen(s), " %d", humidityValue);
                }
                sprintf(s + strlen(s), "]");
                send(s);
                } else if (cb_size(&rh) == 0) { // Sprawdzenie, czy bufor temp jest pusty
                    send("EMP");
                }
        } else if (strcmp(command, "/CLR") == 0) {
                if (strcmp(arguments, "[TMP]") == 0) {
                	cb_init(&temp);
                	sendOk();
                } else if(strcmp(arguments, "[HUM]") == 0) {
                	cb_init(&rh);
                	sendOk();
                } else {
                	send("INC[INCORRECT COMMAND]");
                }
        } else if (strcmp(command, "/BUF") == 0) {
                if (strcmp(arguments, "[TMP]") == 0) {
                	char s[BUFFER_SIZE];
                	sprintf(s, "OKK[buff size: temp %d]", cb_size(&temp));
                	send(s);
                } else if(strcmp(arguments, "[HUM]") == 0) {
                	char s[BUFFER_SIZE];
                	sprintf(s, "OKK[buff size: hum %d]", cb_size(&rh));
                	send(s);
                } else {
                	send("INC[INCORRECT COMMAND]");
                }
        } else if (strcmp(command, "/INT") == 0) {
            if (sscanf(arguments, "HUM,%d", &arg1) == 1 && arg1 >= 20) {
                humInterval = arg1;
                send("OKK[HUM INTERVAL SET]");
            } else if (sscanf(arguments, "TMP,%d", &arg1) == 1 && arg1 >= 20) {
                tempInterval = arg1;
                send("OKK[TEMP INTERVAL SET]");
            } else if (sscanf(arguments, "%d", &arg1) == 1 && arg1 >= 20) {
                tempInterval = humInterval = arg1;
                sendOk();
            }else if (strcmp(command, "INT[CURRENT]") == 0) {
                char response[100];
                snprintf(response, sizeof(response), "OKK[Current Intervals: hum=%d, temp=%d]", humInterval, tempInterval);
                send(response);

            } else {
                send("INC[INCORRECT COMMAND OR INTERVAL]");
            }
        }

    } else {
        send("INC[EMPTY FRAME]");
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
  cb_init(&temp);
  cb_init(&rh);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2,&RX_BF[0],1);
  HAL_TIM_Base_Start_IT(&htim3);
  SysTick_Init(); // Inicjalizacja SysTicka
  DHT11_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  FrameRd();
	  if(dht11_state == DHT11_IDLE)
	  {
	  	 DHT11_Start();
	  }
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D3_Pin|D4_Pin|D5_Pin|D0_Pin
                          |D1_Pin|RS_Pin|E_Pin|RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D3_Pin D4_Pin D5_Pin D0_Pin
                           D1_Pin RS_Pin E_Pin RW_Pin */
  GPIO_InitStruct.Pin = D3_Pin|D4_Pin|D5_Pin|D0_Pin
                          |D1_Pin|RS_Pin|E_Pin|RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
