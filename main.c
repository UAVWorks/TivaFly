#include<stdbool.h>
#include<stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"
#include "inc/hw_adc.h"

#include "drivers/rgb.h"
#include "usb_dev_serial.h"
#include "protocol.h"


#define LED1TASKPRIO 1
#define LED1TASKSTACKSIZE 128
#define PITCH 0
#define ROLL 1
#define YAW 2

//Globales

uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;

QueueHandle_t potQueue;

extern void vUARTTask( void *pvParameters );


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static unsigned char count = 0;

	if (++count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
	while(1);
}



//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

// El codigo de esta tarea esta definida en el fichero command.c, es la que se encarga de procesar los comandos del interprete a traves
// del terminal serie (puTTY)
//Aqui solo la declaramos para poderla referenciar en la funcion main
extern void vUARTTask( void *pvParameters );

// Codigo para procesar los comandos recibidos a traves del canal USB
static portTASK_FUNCTION( CommandProcessingTask, pvParameters ){


	unsigned char frame[MAX_FRAME_SIZE];	//Ojo, esto hace que esta tarea necesite bastante pila
	int numdatos;
	unsigned int errors=0;
	unsigned char command;

	/* The parameters are not used. */
	( void ) pvParameters;

	for(;;)
	{
		numdatos=receive_frame(frame,MAX_FRAME_SIZE);
		if (numdatos>0)
		{	//Si no hay error, proceso la trama que ha llegado.
			numdatos=destuff_and_check_checksum(frame,numdatos);
			if (numdatos<0)
			{
				//Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
				errors++;
				// Procesamiento del error (TODO)
			}
			else
			{
				//El paquete esta bien, luego procedo a tratarlo.
				command=decode_command_type(frame,0);

				switch(command)
				{
				case COMANDO_PING :
					//A un comando de ping se responde con el propio comando
					numdatos=create_frame(frame,command,0,0,MAX_FRAME_SIZE);
					if (numdatos>=0)
					{
						send_frame(frame,numdatos);
					}else{
						//Error de creacion de trama: determinar el error y abortar operacion
						errors++;
						// Procesamiento del error (TODO)
						// Esto de aqui abajo podria ir en una funcion "createFrameError(numdatos)  para evitar
						// tener que copiar y pegar todo en cada operacion de creacion de paquete
						switch(numdatos){
						case PROT_ERROR_NOMEM:
							// Procesamiento del error NO MEMORY (TODO)
							break;
						case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
							// Procesamiento del error STUFFED_FRAME_TOO_LONG (TODO)
							break;
						case PROT_ERROR_COMMAND_TOO_LONG:
							// Procesamiento del error COMMAND TOO LONG (TODO)
							break;
						}
					}
					break;
				case COMANDO_LEDS:         // Comando de ejemplo: eliminar en la aplicacion final
				{
					PARAM_COMANDO_LEDS parametro;
					uint32_t g_ulColors[3] = { 0x0000, 0x0000, 0x0000 };

					if (check_command_param_size(numdatos,sizeof(parametro)))
					{
						extract_packet_command_param(frame,sizeof(parametro),&parametro);
						g_ulColors[0]= parametro.leds.red ? 0x8000 : 0x0000;
						g_ulColors[1]=parametro.leds.green ? 0x8000 : 0x0000;
						g_ulColors[2]= parametro.leds.blue ? 0x8000 : 0x0000;

						RGBColorSet(g_ulColors);
					}
					else
					{
						//Error en estructura de trama
						errors++;
						// Procesamiento del error PROT_ERROR_INCONSISTENT_FRAME_FORMAT (TODO)
					}
				}
				break;
				default:
				{
					PARAM_COMANDO_NO_IMPLEMENTADO parametro;
					parametro.command=command;
					//El comando esta bien pero no esta implementado
					numdatos=create_frame(frame,COMANDO_NO_IMPLEMENTADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
					if (numdatos>=0)
					{
						send_frame(frame,numdatos);
					}
					break;
				}
				}
			}
		}else{ // if (numdatos >0)
			//Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
			errors++;
			// Procesamiento del error (TODO)
		}
	}
}

static portTASK_FUNCTION(SensorTask,pvParameters)
		{

	uint32_t potenciometros[3];
	int16_t ejes[2];
	unsigned char frame[MAX_FRAME_SIZE];

	//
	// Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
	//
	while(1)
	{
		xQueueReceive(potQueue,potenciometros,portMAX_DELAY);
		ejes[PITCH]=(potenciometros[PITCH]*60)/4096-30;
		ejes[ROLL]=(potenciometros[ROLL]*90)/4096-45;
		//ejes[YAW]=(potenciometros[YAW]*180)/4096-90;

		create_frame(frame, COMANDO_EJES, ejes, sizeof(ejes), MAX_FRAME_SIZE);
		send_frame(frame, MAX_FRAME_SIZE);
	}
}


// Rutinas de Interrupcion

void ADCIntHandler();

//Funciones de Configuracion

void confSys();
void confUART();
void confGPIO();
void confTasks();

void confQueue();
void confADC();
void confTimer();


//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{

	confSys();
	confUART();
	confGPIO();
	confADC();
	confQueue();

	//
	// Mensaje de bienvenida inicial.
	//
	UARTprintf("\n\nBienvenido a la aplicacion Simulador Vuelo (curso 2014/15)!\n");
	UARTprintf("\nAutores: Anabel Ramirez y Jose Antonio Yebenes ");



	confTasks();
	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	confTimer();
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}

void confSys(){
	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)


	// Get the system clock speed.
	g_ulSystemClock = SysCtlClockGet();


	//Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
	//                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
	ROM_SysCtlPeripheralClockGating(true);

	// Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
	// Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
	// (y por tanto este no se deberia utilizar para otra cosa).
	CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 3);
}
void confUART(){
	//
	// Inicializa la UARTy la configura a 115.200 bps, 8-N-1 .
	//se usa para mandar y recibir mensajes y comandos por el puerto serie
	// Mediante un programa terminal como gtkterm, putty, cutecom, etc...
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0,115200,SysCtlClockGet());

	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);	//La UART tiene que seguir funcionando aunque el micro este dormido
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);	//La UART tiene que seguir funcionando aunque el micro este dormido

}
void confGPIO(){
	//Inicializa el puerto F (LEDs) --> No hace falta si usamos la libreria RGB
	//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	//    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	//    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);	//LEDS APAGADOS

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	//Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1 (eliminar si no se usa finalmente)
	//RGBInit(1);
	//SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
	//SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
	//SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);	//Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo

}
void confTasks(){
	/**                                              Creacion de tareas 												**/

	// Crea la tarea que gestiona los comandos UART (definida en el fichero commands.c)
	//
	if((xTaskCreate(vUARTTask, (portCHAR *)"Uart", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
	{
		while(1);
	}

	UsbSerialInit(32,32);	//Inicializo el  sistema USB
	//
	// Crea la tarea que gestiona los comandos USB (definidos en CommandProcessingTask)
	//
	if(xTaskCreate(CommandProcessingTask, (portCHAR *)"usbser",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
	{
		while(1);
	}


	if((xTaskCreate(SensorTask, (signed portCHAR *)"Sensor", LED1TASKSTACKSIZE,NULL,tskIDLE_PRIORITY + LED1TASKPRIO, NULL) != pdTRUE))
		{
			while(1);
		}
}

void confADC(){

	IntPrioritySet(INT_ADC0SS0,5);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);   // Habilita ADC0
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);
	ADCSequenceDisable(ADC0_BASE,0); // Deshabilita el secuenciador 1 del ADC0 para su configuracion


	HWREG(ADC0_BASE + ADC_O_PC) = (ADC_PC_SR_500K);	// usar en lugar de SysCtlADCSpeedSet
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);// Disparo de muestreo por instrucciones de Timer
	ADCHardwareOversampleConfigure(ADC0_BASE, 64); //SobreMuestreo de 64 muestras

	// Configuramos los 4 conversores del secuenciador 1 para muestreo del sensor de temperatura
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0); //Sequencer Step 0: Samples Channel PE3
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1); //Sequencer Step 1: Samples Channel PE2
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END); //Sequencer Step 2: Samples Channel PE1

	// Tras configurar el secuenciador, se vuelve a habilitar
	ADCSequenceEnable(ADC0_BASE, 0);
	//Asociamos la función a la interrupción
	ADCIntRegister(ADC0_BASE, 0,ADCIntHandler);
	//Activamos las interrupciones
	ADCIntEnable(ADC0_BASE,0);


}

void confTimer(){
	// Configuracion TIMER0
	// Habilita periferico Timer0
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	// Configura el Timer0 para cuenta periodica de 32 bits (no lo separa en TIMER0A y TIMER0B)
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	uint32_t ui32Period = SysCtlClockGet() *2;
	// Carga la cuenta en el Timer0A
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);
	//Configuramos el Timer como el TRIGGER del ADC
	TimerControlTrigger(TIMER0_BASE,TIMER_A,true);
	// Activa el Timer0A (empezara a funcionar)
	TimerEnable(TIMER0_BASE, TIMER_A);
}

void confQueue(){
		uint32_t potenciometros[3];
		potQueue = xQueueCreate( 1, sizeof(potenciometros) );
}

void ADCIntHandler(){

	ADCIntClear(ADC0_BASE, 0); // Limpia el flag de interrupcion del ADC

	uint32_t potenciometros[3];
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;



	// Tras haber finalizado la conversion, leemos los datos del secuenciador

	 ADCSequenceDataGet(ADC0_BASE, 0, potenciometros);

	 //Enviamos el dato a la tarea
	 xQueueSendFromISR( potQueue,potenciometros,&xHigherPriorityTaskWoken);

	 if(xHigherPriorityTaskWoken == pdTRUE){
		 vPortYieldFromISR();
	 }
}
