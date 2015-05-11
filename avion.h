#ifndef __AVION_H
#define __AVION_H

#include "FreeRTOS.h"
#include "semphr.h"
#include<stdbool.h>
#include<stdint.h>


#define PITCH 0
#define ROLL 1
#define YAW 2

int16_t ejes[3];
float  velocidad;
double combustible;
uint32_t hora;
double  altitud;
int tiempoSim;
bool pilotoAutomatico;


SemaphoreHandle_t EjesSemaphore;
SemaphoreHandle_t VelocidadSemaphore;
SemaphoreHandle_t CombustibleSemaphore;
SemaphoreHandle_t HoraSemaphore;
SemaphoreHandle_t AltitudSemaphore;
SemaphoreHandle_t TiempoSimSemaphore;
SemaphoreHandle_t PilotoAutomaticoSemaphore;

void crearSemaphoresAvion();
void inicializarVariables();

void setEjes(int16_t pitch, int16_t roll, int16_t yaw);
void setVelocidad(float velocidadValor);
void setCombustible(double combustibleValor);
void setHora(uint32_t horaValor);
void setAltitud(double altitudValor);
void setTiempoSim(int tiempoSimValor);
void setPilotoAutomatico(bool pilotoAutomaticoValor);

void getEjes(int16_t *ejes);
float getVelocidad();
double getCombustible();
uint32_t getHora();
double getAltitud();
int getTiempoSim();
bool getPilotoAutomatico();



#endif
