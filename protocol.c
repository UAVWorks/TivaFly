#include<FreeRTOS.h>

#include "crc.h"
#include "usb_dev_serial.h"
#include "protocol.h"

#include <stdlib.h>

extern SendSemaphore;

//Funcion que realiza el stuffining en una trama. 
int frame_stuffing(unsigned char *origen, unsigned char *destino,int longitud, int longitud_maxima)
{
	int i,j;
	unsigned char tmp;
	
	for (i=0,j=0;((i<longitud)&&(j<longitud_maxima));i++,j++)
	{
		tmp=*origen;
		origen++;
		if ((START_FRAME_CHAR==tmp)||(STOP_FRAME_CHAR==tmp)||(ESCAPE_CHAR==tmp))
		{
			*destino=ESCAPE_CHAR;
			destino++;
			j++;
			*destino=tmp^STUFFING_MASK;
			destino++;
		}
		else
		{
			*destino=tmp;
			destino++;
		}
	}
		
	
	if (j>longitud_maxima) 
		return PROT_ERROR_STUFFED_FRAME_TOO_LONG;
	else
		return j;
}

//Funcion para hacer el "Desestuffamiento" de una trama recibida, antes de pasar a procesarla.
int frame_DeStuffing(unsigned char *frame,int longitud)
{
	int i,numStuffedBytes;
	unsigned char *auxptr;
	unsigned char escape_seq_detected=0;
	
	//No hace falta duplicar, se trabaja sobre el propio array, ya que se van eliminando posiciones.
	auxptr=frame;

	for (i=0,numStuffedBytes=0;i<longitud;i++)
	{
		if (*auxptr==ESCAPE_CHAR)
		{
			auxptr++;
			i++;	//Se incremente i
			numStuffedBytes++;
			if ((*auxptr)!=ESCAPE_CHAR)
			{
				*frame=(*auxptr)^STUFFING_MASK;	//Hace el XoR si el siguiente no es tambien 0xFE
				frame++;
				auxptr++;
			}
			else{
				auxptr++;
				escape_seq_detected++;
				//Aqui iria el codigo para tratar una secuencia de escape en caso de querer hacerlo
			}
		}
		else
		{
			*frame=(*auxptr);
			frame++;
			auxptr++;
		}
	}
	
	longitud-=numStuffedBytes;

	return longitud;
}




//Funcion para calcular el checksum y "estuffar" un paquete para su envio
int add_checksum_and_stuff(unsigned char *frame,int size, int max_size)
{
	unsigned short int checksum;
	unsigned char *ptrtochar;
	
	checksum=create_checksum(frame,size);	//Calcula el checksum del paquete antes del Stuffing
	frame[size]=(unsigned char)(checksum&0x0FF);
	frame[size+1]=(unsigned char)(checksum>>8);	//A�ade el checksum
	size+=CHECKSUM_SIZE;

	//Hace una copia temporal del paquete
	//Se hace siempre del mismo tama�o para que no se fragmente el Heap
	ptrtochar=(unsigned char *)pvPortMalloc(max_size);	
	if (!ptrtochar) 
	{
		return PROT_ERROR_NOMEM;
	}
	memcpy(ptrtochar,(void *)frame,size);
	
	size=frame_stuffing(ptrtochar, frame,size, max_size);	//Hace el stuffing	
	vPortFree(ptrtochar);
	
	return (size);
}

//Desestufa y chequea el checksum de un paquete recibido
int destuff_and_check_checksum(unsigned char *frame, int max_size)
{
	unsigned short int checksum,checksum_calc;
	int size; 

	
	
	size=frame_DeStuffing(frame,max_size);
	
	checksum_calc=create_checksum(frame,size-CHECKSUM_SIZE);	//Calcula el checksum del paquete despues del deStuffing
	checksum=(((unsigned short)frame[size-(CHECKSUM_SIZE-1)])<<8)|((unsigned short)frame[size-CHECKSUM_SIZE]);	
	
	if (checksum!=checksum_calc) 
		return PROT_ERROR_BAD_CHECKSUM;
	else
		return (size);
}


//Funcion que crea un comando y lo introduce en una trama
int create_frame(unsigned char *frame, unsigned char command_type, void * param, int param_size, int max_size)
{
	int min_size;
	unsigned char *auxptr;
	int size;
	
	auxptr=frame;
	
	*auxptr=START_FRAME_CHAR;
	auxptr++;
	
	min_size=(MINIMUN_FRAME_SIZE+param_size);	//1 START +1 COMANDO + 2 CHECKSUM +1 STOP
		
	if (min_size>=max_size) 
	{
		return PROT_ERROR_COMMAND_TOO_LONG;
	}
		
	*auxptr=command_type;
	auxptr++;
	if (param_size>0)
	{
		memcpy(auxptr,param,param_size);	//Copia los argumentos
		auxptr+=param_size;
	}
	
	size=add_checksum_and_stuff((frame+START_SIZE),(min_size-(START_SIZE+END_SIZE+CHECKSUM_SIZE)),max_size-(START_SIZE+END_SIZE));	//No se aplica el chesksum y el stuff a los caracteres de inicio y fin
	auxptr=(frame+size+START_SIZE);
	*auxptr=STOP_FRAME_CHAR;
		
	return (size+START_SIZE+END_SIZE);
}

/* Recibe una trama (sin los caracteres de inicio y fin */
/* Utiliza la funcion bloqueante xSerialGetChar ---> Esta funcion es bloqueante y no se puede llamar desde una ISR !!!*/
int receive_frame(unsigned char *frame, int maxFrameSize)
{
	int i;
	unsigned char rxByte;
	
	do 
	{
		//Elimino bytes de la cola de recepcion hasta llegar a comienzo de nueva trama
		xUSBSerialGetChar( ( char *)&rxByte, portMAX_DELAY);
	} while (rxByte!=START_FRAME_CHAR);
	
	i=0;
	do
	{
		xUSBSerialGetChar( ( char *)frame, portMAX_DELAY);
		i++;
	} while ((*(frame++)!=STOP_FRAME_CHAR)&&(i<=maxFrameSize));
	
	if (i>maxFrameSize)
	{
		return PROT_ERROR_RX_FRAME_TOO_LONG;	//Numero Negativo indicando error
	}  
	else
	{
		return (i-END_SIZE);	//Devuelve el numero de bytes recibidos (quitando el de BYTE DE STOP)
	}
}

/* Envia la trama usando la funcion bloqueante xSerialPutChar ---> Esta funcion es bloqueante  y no se puede llamar desde una ISR */
int send_frame(unsigned char *frame, int FrameSize)
{
	int i;
	xSemaphoreTake( SendSemaphore, portMAX_DELAY );
	
	for(i=FrameSize;i>0;i--)
	{
		xUSBSerialPutChar(  *frame++, portMAX_DELAY);
	}
	xSemaphoreGive(SendSemaphore);
	return FrameSize;	
}



void logError(int numdatos){
	switch(numdatos){
		case PROT_ERROR_NOMEM:

				UARTprintf("Error: PROT_ERROR_NOMEM\n ");

			break;
		case PROT_ERROR_STUFFED_FRAME_TOO_LONG:

				UARTprintf("Error: PROT_ERROR_STUFFED_FRAME_TOO_LONG\n ");

			break;
		case PROT_ERROR_COMMAND_TOO_LONG:

				UARTprintf("Error: PROT_ERROR_COMMAND_TOO_LONG\n ");

			break;
		}
}








