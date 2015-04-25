/*
 * usb_dev_serial.h
 *
 *  Created on: 27/01/2013
 *      Author: jmcg
 */

#ifndef USB_DEV_SERIAL_H_
#define USB_DEV_SERIAL_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//Api parecida a la que gestiona el puerto serie desde el fritos.



int UsbSerialInit (  portBASE_TYPE txQueueLength,  portBASE_TYPE rxQueueLength);
 portBASE_TYPE xUSBSerialGetChar (  portCHAR *pcRxedChar, portTickType xBlockTime );
 portBASE_TYPE xUSBSerialPutChar ( portCHAR cOutChar, portTickType xBlockTime );



#endif /* USB_DEV_SERIAL_H_ */
