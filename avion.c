#include "avion.h"
#include "FreeRTOS.h"
#include "semphr.h"

void crearSemaphoresAvion(){
	EjesSemaphore =	 xSemaphoreCreateMutex();
	VelocidadSemaphore=xSemaphoreCreateMutex();
	CombustibleSemaphore=xSemaphoreCreateMutex();
	HoraSemaphore=xSemaphoreCreateMutex();
	AltitudSemaphore=xSemaphoreCreateMutex();
	TiempoSimSemaphore=xSemaphoreCreateMutex();
	PilotoAutomaticoSemaphore=xSemaphoreCreateMutex();
}

void inicializarVariables(){

	setAltitud(3000);
	setCombustible(100);
	setEjes(0,0,0);
	setPilotoAutomatico(false);
	setTiempoSim(1);
	setVelocidad(60);

}


//SET
void setEjes(int16_t pitch, int16_t roll, int16_t yaw){
	xSemaphoreTake(EjesSemaphore, portMAX_DELAY);
		ejes[PITCH]=pitch;
		ejes[ROLL]=roll;
		ejes[YAW]=yaw;
	xSemaphoreGive(EjesSemaphore);
}
void setVelocidad(float velocidadValor){
	xSemaphoreTake(VelocidadSemaphore, portMAX_DELAY);
		velocidad=velocidadValor;
	xSemaphoreGive(VelocidadSemaphore);
}
void setCombustible(double combustibleValor){
	xSemaphoreTake(CombustibleSemaphore, portMAX_DELAY);
		combustible=combustibleValor;
	xSemaphoreGive(CombustibleSemaphore);
}
void setHora(uint32_t horaValor){
	xSemaphoreTake(HoraSemaphore, portMAX_DELAY);
		hora=horaValor;
	xSemaphoreGive(HoraSemaphore);
}
void setAltitud(double altitudValor){
	xSemaphoreTake(AltitudSemaphore, portMAX_DELAY);
		altitud=altitudValor;
	xSemaphoreGive(AltitudSemaphore);
}
void setTiempoSim(int tiempoSimValor){
	xSemaphoreTake(TiempoSimSemaphore, portMAX_DELAY);
		tiempoSim=tiempoSimValor;
	xSemaphoreGive(TiempoSimSemaphore);
}
void setPilotoAutomatico(bool pilotoAutomaticoValor){
	xSemaphoreTake(PilotoAutomaticoSemaphore, portMAX_DELAY);
		pilotoAutomatico=pilotoAutomaticoValor;
	xSemaphoreGive(PilotoAutomaticoSemaphore);
}

//GET
void getEjes(int16_t *ejesReturn){
	xSemaphoreTake(EjesSemaphore, portMAX_DELAY);
		ejesReturn[PITCH]=ejes[PITCH];
		ejesReturn[ROLL]=ejes[ROLL];
		ejesReturn[YAW]=ejes[YAW];
	xSemaphoreGive(EjesSemaphore);
}
float getVelocidad(){
	float velocidadReturn;

	xSemaphoreTake(VelocidadSemaphore, portMAX_DELAY);
		velocidadReturn=velocidad;
	xSemaphoreGive(VelocidadSemaphore);

	return velocidadReturn;
}
double getCombustible(){
	double combustibleReturn;
	xSemaphoreTake(CombustibleSemaphore, portMAX_DELAY);
		combustibleReturn=combustible;
	xSemaphoreGive(CombustibleSemaphore);
	return combustibleReturn;
}
uint32_t getHora(){
	uint32_t horaReturn;

	xSemaphoreTake(HoraSemaphore, portMAX_DELAY);
		horaReturn=hora;
	xSemaphoreGive(HoraSemaphore);

	return horaReturn;
}
double getAltitud(){
	double altitudReturn;
	xSemaphoreTake(AltitudSemaphore, portMAX_DELAY);
		altitudReturn=altitud;
	xSemaphoreGive(AltitudSemaphore);
	return altitudReturn;
}
int getTiempoSim(){
	int tiempoSimReturn;
	xSemaphoreTake(TiempoSimSemaphore, portMAX_DELAY);
		tiempoSimReturn=tiempoSim;
	xSemaphoreGive(TiempoSimSemaphore);
	return tiempoSimReturn;
}
bool getPilotoAutomatico(){
	bool pilotoReturn;
	xSemaphoreTake(PilotoAutomaticoSemaphore, portMAX_DELAY);
		pilotoReturn=pilotoAutomatico;
	xSemaphoreGive(PilotoAutomaticoSemaphore);
	return pilotoReturn;
}
