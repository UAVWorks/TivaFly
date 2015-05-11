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
#include "FreeRTOS/source/include/event_groups.h"

#include "drivers/rgb.h"
#include "usb_dev_serial.h"
#include "protocol.h"

#include <cmath>

#include "avion.h"

#define LED1TASKPRIO 1
#define LED1TASKSTACKSIZE 128


//Globales

uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;

QueueHandle_t potQueue;
QueueHandle_t velocidadQueue;

SemaphoreHandle_t SendSemaphore=NULL;

// Punteros a las tareas
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t consumoTaskHandle = NULL;
TaskHandle_t PilautTaskHandle = NULL;
TaskHandle_t altitudTaskHandle = NULL;
TaskHandle_t turbulenciasTaskHandle = NULL;

EventGroupHandle_t xEventGroup;
#define TrazaBit	( 1 << 0 )
#define PilotoAutomaticoBit (1 << 1)

extern void vUARTTask( void *pvParameters );

uint32_t color[3];

bool pulsacionLarga;




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



static portTASK_FUNCTION(PilAuto,pvParameters)
{

	unsigned char frame[MAX_FRAME_SIZE];
	int num_datos;


	//Desactivamos las lecturas del ADC
	ADCSequenceDisable(ADC0_BASE,0);


	bool pilotoAutomatico =true;
	EventBits_t bits;

	// Enviamos la trama para indicar a QT que estamos con el piloto Automatico
	num_datos=create_frame(frame, COMANDO_AUTOMATICO, &pilotoAutomatico, sizeof(pilotoAutomatico), MAX_FRAME_SIZE);
	if (num_datos>=0){
		send_frame(frame, num_datos);
	}else{

		logError(num_datos);

	}


	while(1)
	{
		vTaskDelay(configTICK_RATE_HZ); // Esperamos 1 seg

		bits=xEventGroupGetBits(xEventGroup); // Leemos los flags

		if(bits & PilotoAutomaticoBit != PilotoAutomaticoBit){  //Si el flags de pilotoAutomatico esta a cero quitamos el pilotoAutomatico
			pilotoAutomatico=false;
			num_datos=create_frame(frame, COMANDO_AUTOMATICO, &pilotoAutomatico, sizeof(pilotoAutomatico), MAX_FRAME_SIZE);
			if (num_datos>=0){
				send_frame(frame, num_datos);

			}else{

				logError(num_datos);

			}
			ADCSequenceEnable(ADC0_BASE,0); // Habilitamos el ADC
			vTaskDelete(NULL);
		}


		//Centramos el avion, ponemos la velocidad a 100 y lo mandamos a QT
		setEjes(0,0,0);
		setVelocidad(100.0f);
		num_datos=create_frame(frame, COMANDO_EJES, &ejes, sizeof(ejes), MAX_FRAME_SIZE);
			if (num_datos>=0){
				send_frame(frame, num_datos);
			}else{

				logError(num_datos);

			}
		num_datos=create_frame(frame, COMANDO_SPEED, &velocidad, sizeof(velocidad), MAX_FRAME_SIZE);
			if (num_datos>=0){
				send_frame(frame, num_datos);
			}else{

				logError(num_datos);

			}



	}
}

static portTASK_FUNCTION(turbulenciasTask,pvParameters)
{

	unsigned char frame[MAX_FRAME_SIZE];
	int num_datos;
	int parametro=0;
	int variacion=0;

	int16_t ejes[3];

	while(1)
	{
		//Esperamos un tiempo entre 1 y 5 seg
		vTaskDelay(configTICK_RATE_HZ*(rand()%5+1));

		// Obtenemos que parametro y cuanto porcentaje vamos a variarlo
		parametro=rand()%3;
		variacion=rand()%11-5;

		getEjes(ejes);

		switch(parametro){
			case PITCH:
				ejes[PITCH]+= 45*variacion/100;
				break;
			case ROLL:
				ejes[ROLL]+= 30*variacion/100;
				break;
			case YAW:
				ejes[YAW]+= 180*variacion/100;
				break;
		}

		//Enviamos los cambios
		setEjes(ejes[PITCH], ejes[ROLL], ejes[YAW]);

		num_datos=create_frame(frame, COMANDO_EJES, &ejes, sizeof(ejes), MAX_FRAME_SIZE);
		if (num_datos>=0){
			send_frame(frame, num_datos);
		}else{

			logError(num_datos);

		}


	}
}



static portTASK_FUNCTION(HighTask,pvParameters)
{

	unsigned char frame[MAX_FRAME_SIZE];
	int num_datos;

	double altitud;
	int16_t ejes[3];
	double combustible;
	float velocidad;
	int tiempoSim;

	while(1)
	{
		vTaskDelay(configTICK_RATE_HZ); //Cada seg

		//Obtenemos las variables
		altitud=getAltitud();
		velocidad=getVelocidad();
		tiempoSim=getTiempoSim();

		if(altitud>0){ //Si la altitud es mayor que cero cambiamos la altitud

			getEjes(ejes);
			altitud += sin((ejes[PITCH]*3.14f)/180) *(velocidad*(1000/(60/tiempoSim)));

			if(altitud>99999){ //99999 es el maximo del panel QT
				altitud=99999;
			}

			//Enviamos la trama con la altitud
			setAltitud(altitud);

			num_datos=create_frame(frame, COMANDO_HIGH, &altitud, sizeof(altitud), MAX_FRAME_SIZE);
			if (num_datos>=0){
				send_frame(frame, num_datos);
			}else{

				logError(num_datos);

			}

			//Obtenemos el combustible
			combustible=getCombustible();
			if(combustible==0){//Si ya estamos sin combustible hacemos parpadear los led con la altitud
				color[BLUE]=0xFFFF;
				if(altitud<=800){
					color[RED]=0xFFFF;
					color[GREEN]=0x0;
				}
				RGBColorSet(color);
				if(altitud<2000){
					RGBBlinkRateSet((float)(1000/(altitud+1)));
				}
				tiempoSim=getTiempoSim();

				//Aumentamos la velocidad a 9,8m/s^2
				velocidad+=9.8*(60*tiempoSim)/1000;
				setVelocidad(velocidad);
				//Enviamos la nueva velocidad
				num_datos=create_frame(frame, COMANDO_SPEED, &velocidad, sizeof(velocidad), MAX_FRAME_SIZE);
				if (num_datos>=0){
					send_frame(frame, num_datos);
				}else{

					logError(num_datos);

				}
			}

		}else{ //Si la altitud es menor o igual a 0

			//Variables a cero
			velocidad=0;
			altitud=0;

			setVelocidad(velocidad);
			setAltitud(altitud);

			//Enviamos comando colision
			if(combustible!=0) ADCSequenceDisable(ADC0_BASE,0);
			num_datos=create_frame(frame, COMANDO_COLISION,NULL, 0, MAX_FRAME_SIZE);
			if (num_datos>=0){
				send_frame(frame, num_datos);
			}else{

				logError(num_datos);

			}


			vTaskEndScheduler(); //Salimos del Scheduler de FreeRTOS

		}




	}
}

static portTASK_FUNCTION(SensorTask,pvParameters)
		{

	uint32_t potenciometros[3];

	int16_t lastEjes[3]={0,0,0};

	unsigned char frame[MAX_FRAME_SIZE];
	int num_datos;

	int16_t ejes[3];

	getEjes(ejes);

	// Enviamos incialmente el estado de los ejes
	num_datos=create_frame(frame, COMANDO_EJES, ejes, sizeof(ejes), MAX_FRAME_SIZE);
	if (num_datos>=0){
	send_frame(frame, num_datos);
	}else{

		logError(num_datos);

	}

	while(1)
	{
		xQueueReceive(potQueue,potenciometros,portMAX_DELAY); //Esperamos a que llegue información del ADC

		getEjes(ejes); //Obtenemos como están actualmente los ejes

		/* Para el PITCH y ROLL los potenciometros tendran cinco zonas donde la central no modificará los ejes y
		 * las laterales aumentaran o disminuiran los valores de uno en uno o dos en dos.
		 */
		if(potenciometros[PITCH]<819){
			ejes[PITCH]-=2;
		}else if(potenciometros[PITCH]>=819 && potenciometros[PITCH]<1638){
			ejes[PITCH]-=1;
		}else if(potenciometros[PITCH]>=2458 && potenciometros[PITCH]<3276){
			ejes[PITCH]+=1;
		}else if(potenciometros[PITCH]>=3276 && potenciometros[PITCH]<4095){
			ejes[PITCH]+=2;
		}

		if( potenciometros[ROLL]<819){
			ejes[ROLL]-=2;
		}else if(potenciometros[ROLL]>=819 && potenciometros[ROLL]<1638){
			ejes[ROLL]-=1;
		}else if(potenciometros[ROLL]>=2458 && potenciometros[ROLL]<3276){
			ejes[ROLL]+=1;
		}else if(potenciometros[ROLL]>=3276 && potenciometros[ROLL]<4095){
			ejes[ROLL]+=2;
		}

		//Vemos si ha llegado al máximo
		if (ejes[PITCH]>45){
			ejes[PITCH]=45;
		}else if (ejes[PITCH]<-45){
			ejes[PITCH]=-45;
		}

		if (ejes[ROLL]>30){
			ejes[ROLL]=30;
		}else if (ejes[ROLL]<-30){
			ejes[ROLL]=-30;
		}


		ejes[YAW]=(potenciometros[YAW]*360)/4095; //Obtenemos el valor del yaw entre 0 y 360

		setEjes(ejes[PITCH], ejes[ROLL], ejes[YAW]); //Modificamos los ejes

		if(ejes[PITCH]!=lastEjes[PITCH] || ejes[ROLL]!=lastEjes[ROLL] || ejes[YAW]!=lastEjes[YAW]){
			//Solo enviamos los ejes si se han modificado
			num_datos=create_frame(frame, COMANDO_EJES, ejes, sizeof(ejes), MAX_FRAME_SIZE);
			if (num_datos>=0){
			send_frame(frame, num_datos);
			}else{

				logError(num_datos);

			}
			lastEjes[PITCH]=ejes[PITCH];
			lastEjes[ROLL]=ejes[ROLL];
			lastEjes[YAW]=ejes[YAW];
		}
	}

}

static portTASK_FUNCTION(TimeTask,pvParameters)
{
	unsigned char frame[MAX_FRAME_SIZE];
	int num_datos;
	int tiempoSim;
	uint32_t hora;

	while(1)
	{
		vTaskDelay(configTICK_RATE_HZ); //cada segundo

		tiempoSim=getTiempoSim(); //Obtenemos la equivalencia de 1 segundo Real son tiempoSim minutos en el simulado
		hora = getHora(); //Obtenemos la hora

		hora+=tiempoSim; //aumentamos hora tiempoSim (Minutos)
		if(hora>1440){ //si llegamos a las 24:00 (1440 minutos) ponemos las 00:00
			hora=0;
		}

		//Modificamos y enviamos la hora
		setHora(hora);

		num_datos=create_frame(frame, COMANDO_TIME, &hora, sizeof(hora), MAX_FRAME_SIZE);
		if (num_datos>=0){
			send_frame(frame, num_datos);
		}else{

			logError(num_datos);

		}

	}
}

static portTASK_FUNCTION(ConsumoTask,pvParameters)
{

	double consumo=0.0374*exp(0.02*((velocidad*100)/240)); //actualizamos el consumo


	TickType_t tiempo_ant =xTaskGetTickCount(  ); //obtenemos los tick transcurridos


	unsigned char frame[MAX_FRAME_SIZE];
	int num_datos;

	double combustible;
	int16_t ejes[3];
	double  altitud;

	while(1)
	{

		if(xQueueReceive(velocidadQueue,&velocidad, configTICK_RATE_HZ)){
			//Si recibimos un nuevo valor de velocidad cambiamos brillo del led azul
			color[BLUE]=0xFFFF;
			RGBSet(color,((float)velocidad)/241);
		}

		if((xTaskGetTickCount(  )-tiempo_ant)>=configTICK_RATE_HZ*(60/tiempoSim) && combustible!=0){
			//Cada minuto real (1 hora simulada)
			combustible=getCombustible();
			//Modificamos el combustible segun el consumo
			combustible -= 0.5*exp(0.02*(velocidad*100/240)) ;


			if(combustible<=20){ //Encendemos el Led Verde si el combustible es menor que 20
				color[GREEN]=0xFFFF;
				RGBColorSet(color);
			}

			if(combustible<=0){ //Si el combustible es cero desactivamos el ADC
				combustible=0;
				velocidad=0;
				color[BLUE]=0x0;

				xEventGroupClearBits( xEventGroup, PilotoAutomaticoBit );
				ADCSequenceDisable(ADC0_BASE,0);

			}

			//Enviamos el combustible
			setCombustible(combustible);
			num_datos=create_frame(frame, COMANDO_FUEL, &combustible, sizeof(combustible), MAX_FRAME_SIZE);
			if (num_datos>=0){
				send_frame(frame, num_datos);
			}else{

				logError(num_datos);

			}

			tiempo_ant =xTaskGetTickCount(  );
		}


		getEjes(ejes);
		altitud=getAltitud();
		if(ejes[PITCH]>-45 && combustible==0 && altitud>0){ //si el combustible es cero ponemos PITCH =45 poco a poco

			ejes[PITCH]--;

			setEjes(ejes[PITCH],ejes[ROLL],ejes[YAW]);

			num_datos=create_frame(frame, COMANDO_EJES, ejes, sizeof(ejes), MAX_FRAME_SIZE);
			if (num_datos>=0){
				send_frame(frame, num_datos);
			}else{

				logError(num_datos);

			}
		}



	}
}

// Codigo para procesar los comandos recibidos a traves del canal USB
static portTASK_FUNCTION( CommandProcessingTask, pvParameters ){



	unsigned char frame[MAX_FRAME_SIZE];	//Ojo, esto hace que esta tarea necesite bastante pila
	int numdatos;
	unsigned int errors=0;
	unsigned char command;
	EventBits_t bits;



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
				bits=xEventGroupGetBits(xEventGroup);
				switch(command)
				{
				case COMANDO_PING :

					if(bits & TrazaBit == TrazaBit){
						UARTprintf("Comando PING\n ");
					}

					//A un comando de ping se responde con el propio comando
					numdatos=create_frame(frame,command,0,0,MAX_FRAME_SIZE);
					if (numdatos>=0)
					{
						send_frame(frame,numdatos);
					}else{
						//Error de creacion de trama: determinar el error y abortar operacion
						errors++;
						logError(numdatos);

					}
					break;
				case COMANDO_START:         // Comando de ejemplo: eliminar en la aplicacion final
				{

					if(bits & TrazaBit == TrazaBit){
						UARTprintf("Comando START\n ");
					}


					if(sensorTaskHandle == NULL){

						inicializarVariables();

						if((xTaskCreate(ConsumoTask, (signed portCHAR *)"Consumo", LED1TASKSTACKSIZE,NULL,tskIDLE_PRIORITY + 1, &consumoTaskHandle)!= pdTRUE))
						{
							while(1);
						}

						if((xTaskCreate(SensorTask, (signed portCHAR *)"Sensor", LED1TASKSTACKSIZE,NULL,tskIDLE_PRIORITY + 1, &sensorTaskHandle) != pdTRUE))
						{
							while(1);
						}
						if((xTaskCreate(HighTask, (signed portCHAR *)"Altitud", LED1TASKSTACKSIZE,NULL,tskIDLE_PRIORITY + 1, &altitudTaskHandle) != pdTRUE))
						{
							while(1);
						}

						if((xTaskCreate(turbulenciasTask, (signed portCHAR *)"Turbulencias", LED1TASKSTACKSIZE,NULL,tskIDLE_PRIORITY + 1, &turbulenciasTaskHandle) != pdTRUE))
						{
							while(1);
						}
					}

				}
				break;
				case COMANDO_STOP:
				{
					if(bits & TrazaBit == TrazaBit){
						UARTprintf("Comando STOP\n ");
					}
					if(combustible>0){ //Eliminamos las tareas en el STOP
						vTaskDelete(sensorTaskHandle);
						vTaskDelete( consumoTaskHandle );
						vTaskDelete(altitudTaskHandle);
						vTaskDelete( turbulenciasTaskHandle );

					}

				}
				break;
				case COMANDO_SPEED:
				{

					if(bits & TrazaBit == TrazaBit){
						UARTprintf("Comando SPEED\n ");
					}
					float  velocidad;

					//Recibimos y enviamos por la cola la velocidad
					extract_packet_command_param(frame,sizeof(velocidad),&velocidad);
					xQueueSend( velocidadQueue,&velocidad,portMAX_DELAY);


				}
				break;
				case COMANDO_TIME:
				{

					if(bits & TrazaBit == TrazaBit){
						UARTprintf("Comando TIME\n ");
					}

					uint32_t hora;
					extract_packet_command_param(frame,sizeof(hora),&hora);
					//recibimos y actualizamos el valor de Hora
					setHora(hora);
					//Creamos la tarea Time
					if(xTaskCreate(TimeTask, (portCHAR *)"Time",LED1TASKSTACKSIZE, NULL, tskIDLE_PRIORITY + 1, NULL) != pdTRUE)
					{
						while(1);
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

		}
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
void confSemaphores();
void confADC();
void confTimer();
void ButtonHandler();
void timerBotonHandler();


//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{

	//Establecemos el color del led a apagado
	color[0]=0x0;
	color[1]=0x0;
	color[2]=0x0;


	confSys();
	confUART();
	confGPIO();
	confADC();
	confQueue();
	confSemaphores();

	//Creamos los flags
	xEventGroup = xEventGroupCreate();
	xEventGroupClearBits( xEventGroup, TrazaBit | PilotoAutomaticoBit );


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
	 //   ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	 // ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	//ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);	//LEDS APAGADOS

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);



	//Inicializa los LEDs usando libreria RGB
	RGBInit(1);
	SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
	SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
	RGBEnable();

	//Inicializamos los botones y su interrupción
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ButtonsInit();
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0|GPIO_INT_PIN_4);
	GPIOIntRegister(GPIO_PORTF_BASE,ButtonHandler);
	GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_INT_PIN_0|GPIO_INT_PIN_4, GPIO_BOTH_EDGES);
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0|GPIO_INT_PIN_4);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);


}
void confTasks(){
	// Crea la tarea que gestiona los comandos UART (definida en el fichero commands.c)
	//
	if((xTaskCreate(vUARTTask, (portCHAR *)"Uart", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
	{
		while(1);
	}

	UsbSerialInit(32,32);	//Inicializo el  sistema USB

	if(xTaskCreate(CommandProcessingTask, (portCHAR *)"usbser",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
	{
		while(1);
	}

}

void confADC(){



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

	IntPrioritySet(INT_ADC0SS0,5<<5);
	// Tras configurar el secuenciador, se vuelve a habilitar
	ADCSequenceEnable(ADC0_BASE, 0);
	//Asociamos la funcion a la interrupcion
	ADCIntRegister(ADC0_BASE, 0,ADCIntHandler);

	//Activamos las interrupciones
	ADCIntEnable(ADC0_BASE,0);


}

void confTimer(){

	//Inicializamos el TIMER2 para el ADC como Trigger a una frecuencia de 10Hz
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);
	TimerControlStall(TIMER2_BASE,TIMER_A,true);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	uint32_t ui32Period = SysCtlClockGet() *0.1;
	TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period -1);
	TimerControlTrigger(TIMER2_BASE,TIMER_A,true);
	TimerEnable(TIMER2_BASE, TIMER_A);

	//Inicializamos el TIMER4 para la pulsación larga con un periodo de 2 segundos
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
  SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER4);
  TimerConfigure(TIMER4_BASE, TIMER_CFG_ONE_SHOT);
  ui32Period = (SysCtlClockGet() *2) ;
  TimerLoadSet(TIMER4_BASE, TIMER_A, ui32Period -1);
  IntEnable(INT_TIMER4A);
  TimerIntRegister(TIMER4_BASE,TIMER_A,timerBotonHandler);
  IntPrioritySet(INT_TIMER4A,5<<5);
  TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

}

void confQueue(){

		//Cremoas las colas necesarias
		uint32_t potenciometros[3];
		potQueue = xQueueCreate( 1, sizeof(potenciometros) ); //Cola para el ADC

		velocidadQueue=xQueueCreate(1,sizeof(uint32_t)); //Cola para la velocidad
}

void confSemaphores(){

	//Creamos los mutex
	SendSemaphore = xSemaphoreCreateMutex(); //mutex para el envio de comandos

	crearSemaphoresAvion(); //Mutexes proteccion de variables
}

void ADCIntHandler(){

	ADCIntClear(ADC0_BASE, 0); // Limpia el flag de interrupcion del ADC
	uint32_t potenciometros[3];
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	// leemos los datos del secuenciador
	 ADCSequenceDataGet(ADC0_BASE, 0, potenciometros);

	 //Enviamos el dato a la tarea
	 xQueueSendFromISR( potQueue,potenciometros,&xHigherPriorityTaskWoken);
	 if(xHigherPriorityTaskWoken == pdTRUE){
		 vPortYieldFromISR();
	 }

}

void ButtonHandler(){
	uint32_t mask=GPIOIntStatus(GPIO_PORTF_BASE,false);

	uint8_t value=0;

	if(mask & GPIO_PIN_4){
		//Boton izquierdo
		value= GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
		if(value==0){
			//boton pulsado
			// Activa el Timer4A (empezara a funcionar)
			TimerEnable(TIMER4_BASE, TIMER_A);
			pulsacionLarga=true;

		}else{
			TimerDisable(TIMER4_BASE,TIMER_A);

			if(pulsacionLarga){
				xEventGroupSetBits(xEventGroup, PilotoAutomaticoBit);
				if((xTaskCreate(PilAuto, (signed portCHAR *)"Piloto Auto", LED1TASKSTACKSIZE,NULL,tskIDLE_PRIORITY + 1, &PilautTaskHandle) != pdTRUE))
				{
					while(1);
				}
			}
		}

	}

	if(mask & GPIO_PIN_0){
		//boton derecho
		xEventGroupClearBits( xEventGroup, PilotoAutomaticoBit );
	}

	GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);
}

void timerBotonHandler(){

	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

	combustible=100;
	pulsacionLarga=false;
}
