/*
 * Lab_6_1_starter.c
 *
 * Created: 15/5/2019
 *  Author: vmon0003
 */ 


#include <asf.h>
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "FastPID.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "FastPID.h"


#define MY_PHOTOELECTRIC	PIO_PC24_IDX //ARDUINO DUE DIGITAL PIN 6

#define MAX_MOTOR_RPM 500

// Motor constants
#define MOTOR_RATED_RPM 430
#define COUNT_PER_REVOLUTION (98*2) // Encoder is specified as 98 pulses per revolution. Hardware QDEC seems to accumulate both A and B pulses
uint32_t g_SetpointRPM = 0;
uint32_t g_direction = 1;
uint32_t slit_time = 20;
uint32_t g_pwm = 0;

/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    20

#define MOTOR_DIR_PIN PIO_PC26_IDX // pin 4 on the board

// PID parameters
//float Kp=0.5, Ki=0.5, Kd=0, Hz=10;
float Kp=2, Ki=0.001, Kd=1, Hz=10;
int output_bits = 16;
bool output_signed = true;

// global variable 
uint32_t half_period_global = 0;
uint32_t previousMilli_global = 0;

// Function prototypes
static void configure_console(void);
static void vHandlerTask( void *pvParameters );
void photoelectric_handler(const uint32_t id, const uint32_t index);
void motor_control_task(void *);
void look_up_table_task(void *);

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
xSemaphoreHandle xBinarySemaphore;

static pwm_channel_t motor_pwm_config = {
	.alignment = PWM_ALIGN_LEFT,
	.polarity = PWM_LOW,
	.ul_prescaler = PWM_CMR_CPRE_CLKA,
	.ul_period = PERIOD_VALUE,
	.ul_duty = INIT_DUTY_VALUE,
	.channel = PWM_CHANNEL_6
};

int main(void)
{
	int i = 0;
	uint32_t val;
	//memset(gs_s_adc_values, 0x0, BUFFER_SIZE * sizeof(int16_t));

	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	ioport_set_pin_dir(MY_PHOTOELECTRIC, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(MY_PHOTOELECTRIC, IOPORT_MODE_PULLUP);
	
	// Pin 4, motor dir
	ioport_set_pin_dir(MOTOR_DIR_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(MOTOR_DIR_PIN, 0); // Fix the initial direction for the motor motion

	configure_console();
	
	// Initialize the Fast PID controller based on the given parameters.
	InitializeFastPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

	/* PHOTO ELECTRIC INTERRUPT */
	pmc_enable_periph_clk(ID_PIOC);
	pio_set_input(PIOC, PIO_PC24, PIO_PULLUP);
	pio_handler_set(PIOC, ID_PIOC, PIO_PC24, PIO_IT_EDGE, photoelectric_handler); 
	
	// Setup PWM
	pmc_enable_periph_clk(ID_PWM);
	pwm_channel_disable(PWM, PWM_CHANNEL_6); // PWM channel 6 is on digital pin 7 (PC23)
	pwm_clock_t pwm_clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE, //1Khz frequency = 1 ms steps
		.ul_clkb = 0,
		.ul_mck =  sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &pwm_clock_setting);
	pwm_channel_init(PWM, &motor_pwm_config);
	pwm_channel_enable(PWM, PWM_CHANNEL_6);

	// Configure Timer Counter for quadrature decoding
	// Uses TIOA0 and TIOB0 which are on pins 2 and 13
	pmc_enable_periph_clk(ID_TC0);
	tc_init(TC0, 0, TC_CMR_ETRGEDG_RISING | TC_CMR_ABETRG | TC_CMR_TCCLKS_XC0);
	tc_set_block_mode(TC0, TC_BMR_QDEN | TC_BMR_POSEN | TC_BMR_MAXFILT(63));
	
	/* Before a semaphore is used it must be explicitly created.  In this example
    a binary semaphore is created. */
    vSemaphoreCreateBinary( xBinarySemaphore );

	// create the task 
	BaseType_t res;
	res =xTaskCreate(motor_control_task, "Motor Control", 512, NULL, 1, NULL); // lowest priority task
	res =xTaskCreate(look_up_table_task, "look_up_table_task", 512, NULL, 2, NULL); // highest priority task
	
	if(res == pdPASS ) 
	{
		tc_start(TC0, 0);	// start the tc counter 
		
		// enable the photoelectric sensor ISR 
		pio_enable_interrupt(PIOC, PIO_PC24);
		NVIC_SetPriority(PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
		NVIC_EnableIRQ(PIOC_IRQn);

		vTaskStartScheduler();
	}

	for( ;; );
	return 0;
}

void look_up_table_task(void *pvParameters) {
	TickType_t last_wake_time = xTaskGetTickCount();
	int32_t g_lookup_Pos [38] = {45,41,40,33,33,33,33,33,30,30,30,30,30,28,28,23,23,23,23,20,20,20,20,20,20,15,15,15,15,15,15,15,13,13,13,13,13};
	int32_t g_tickShort [38] = {5,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,32,33,34,35,36,39,40,42,44,47,48,52,54};
	int32_t g_tickPWM [38] = {15,15,15,15,14,14,14,12,12,12,10,10,10,10,10,9,9,9,9,8,8,8,8,8,8,8,8,8,7,7,7,7,6,6,6,6,6,6};
	
	xSemaphoreTake( xBinarySemaphore, 0 );
	
	while (1) {
		xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );
		
		for (int i = 0; i < 38; i++) // Run through the look up table 
		{
			if (slit_time < g_tickShort[i]) // obtain the first index which the slit time is smaller than the element in lookup table
			{
				g_SetpointRPM = g_direction * g_lookup_Pos[i]; // the exact position to go to (with magnitude and direction)
				g_pwm = g_tickPWM[i];
				break;
			}
		}
		
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
	}
}

void motor_control_task(void *pvParameters) {
	TickType_t last_wake_time = xTaskGetTickCount();
	int32_t motor_pos;
	int16_t pid_output;
	int flag = 0;
	pid_output = step(g_SetpointRPM, motor_pos);
	int PID_threshold = 8;
	static uint32_t currentMillis = 0;
	static uint32_t previousMillis = 0;
	static uint32_t duration = 0;
	
	while (1) {
		
		motor_pos = tc_read_cv(TC0, 0); // get the current position of the motor 
		currentMillis = xTaskGetTickCount();

		duration = currentMillis - previousMilli_global; //keep track how long has this motor monitoring been running
		
		if (duration < (half_period_global)/2) 
		{
			pid_output = step(g_SetpointRPM, motor_pos);
			//printf("hello\n");
		}
		else
		{
			// if this motor monitoring task has been running for > quarter period, force it back to position 0
			pid_output = step(0, motor_pos);
			//printf("duration in task = %d\n", duration);
		}

		//printf("Motor pos: %d  |  Adc value: %d  |  Output from PID is : %d\n", motor_pos, g_SetpointRPM, pid_output);

		if (pid_output > PID_threshold)
		// means the desired location > current location --> continue forward
		{
			ioport_set_pin_level(MOTOR_DIR_PIN, 0);
			pwm_channel_update_duty(PWM, &motor_pwm_config, g_pwm); //Actual updating the PWM channel duty cycle (to output PIN 7)
		}
		else if (pid_output < -PID_threshold)
		// means the desired location < current location --> reverse
		{
			ioport_set_pin_level(MOTOR_DIR_PIN, 1);
			pwm_channel_update_duty(PWM, &motor_pwm_config, g_pwm); //Actual updating the PWM channel duty cycle (to output PIN 7)
		}
		else
		// once the PID output is smaller than threshold --> reached our desired location --> stop
		{
			pwm_channel_update_duty(PWM, &motor_pwm_config, 0); //Stop the motor
		}
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
	}
}

void photoelectric_handler(const uint32_t id, const uint32_t index)
{
	static bool previousState = LOW;
	static uint32_t prevMillis = 0;
	static uint32_t currentMillis = 0;
	
	bool currentState = LOW;
	static uint32_t duration = 0;
	static uint32_t duration_1 = 0;
	static uint32_t duration_2 = 0;
	static uint32_t half_period = 0;
	static uint32_t first_approaching = 0;
	static uint32_t second_approaching = 0;
	static uint32_t rpm = 0;
	static int16_t output = 0;
	static uint32_t dutyCycle = 0;
	unsigned int pinVal = 0;
	static int rise_flag = 0;
	
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	
	static int out_counter = 0;

	if ((id == ID_PIOC) && (index == PIO_PC24)){
		// ensuring the interrupt is raised by the appropriate pin (PIN 6) -- photoelectric 
		pinVal = ioport_get_pin_level(MY_PHOTOELECTRIC); // read the current signal value from pin7
		
		if (pinVal==1 && rise_flag == 0) // first rising edge 
		{
			first_approaching = xTaskGetTickCountFromISR();
			half_period =  first_approaching - second_approaching;
			second_approaching = first_approaching;
			
			currentMillis = xTaskGetTickCountFromISR();
			prevMillis = currentMillis; // update the previous timestamp for next iteration
			
			if (half_period < 900) // a sanity check (to ensure the half period is not too long)
			{
				if(duration_1 > duration_2) // compare duration 1 & duration 2 to get the pendulum's direction 
				{
					printf("duration 1 : %d\n", duration_1);
					printf("duration 2 : %d\n", duration_2);
					printf("Left\n");
					//printf("half period = %d\n", half_period);
					g_direction = -1; // going to the left 
				}
				else
				{
					printf("duration 1 : %d\n", duration_1);
					printf("duration 2 : %d\n", duration_2);
					printf("Right\n");
					//printf("half period = %d\n", half_period);
					g_direction = 1; // going to the right
				}
				
				// passing the half period and second approaching time to global variable 
				// for motor monitoring task 
				half_period_global = half_period ;
				previousMilli_global = second_approaching; 
			}
			
		}
		else if (pinVal == 0 && rise_flag == 0) // first falling edge 
		{
			currentMillis = xTaskGetTickCountFromISR();
			duration_1 = (currentMillis - prevMillis); // duration in minutes 
			prevMillis = currentMillis; // update the previous timestamp for next iteration 
			rise_flag = 1;
		}
		else if (pinVal==1 && rise_flag == 1) //second rising edge 
		{
			currentMillis = xTaskGetTickCountFromISR();
			prevMillis = currentMillis; // update the previous timestamp for next iteration
		}
		else if (pinVal==0 && rise_flag == 1)
		{
			currentMillis = xTaskGetTickCountFromISR();
			duration_2 = (currentMillis - prevMillis); // duration in minutes
			prevMillis = currentMillis; // update the previous timestamp for next iteration
			rise_flag = 0;
		}
		
		if (duration_2 > duration_1) // just to get the T_short (smallest between "duration_1" and "duration_2" to lookup table task
		{
			slit_time = duration_1;
			xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken ); // give the "key" so that "lookup table task" can be unblocked
		}
		else
		{
			slit_time = duration_2;
			xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken ); // give the "key" so that "lookup table task" can be unblocked
		}
		
	}
	
	// to invoke PendSV to immediately switch to "TaskHandler" without waiting to wait for systick interrupt
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	
	return;

}

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);
}


void vApplicationIdleHook( void )
{
}
void vApplicationMallocFailedHook( void )
{
	for( ;; );
}
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	for( ;; );
}
void vApplicationTickHook( void )
{
}