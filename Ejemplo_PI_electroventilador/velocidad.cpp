/******************************************************************************
* Velocidad de cambios de estado
*******************************************************************************
* Archivo: velocidad.cpp
* Resumen: El módulo reúne algunas funciones para calcular la velocidad
*          con que un pin cambia de estado.
* Autor:   Guillermo Caporaletti <gfcaporaletti@undav.edu.ar>
* Fecha:   Abril de 2023
******************************************************************************/

/******************************************************************************
* Librerías y módulos incluidas
******************************************************************************/

#include "velocidad.h"

/******************************************************************************
* Variables privadas (para manejo de la ISR)
******************************************************************************/

byte PinEntrada;                      // Pin de entrada cuya velocidad de cambio de estado deseamos medir 
volatile unsigned long usAnterior=0;  // Almacena el último tiempo de la interrupción en microsegundos.
volatile unsigned long usDelta=0;     // Almacena cuánto pasó entre cada interrupción en microsegundos.
volatile unsigned long contadorISR=0; // Cuenta cambios de estado: otro modo de registro.
volatile bool estadoSensor;           // Guarda estado último para evitar falsas detecciones de cambios de estado.
int velocidadAnterior = 0;            // Esto puede usarse para promediar las velocidades medidas

/******************************************************************************
* Funciones privadas
******************************************************************************/

// ISR que calcula cuánto pasó desde la anterior interrupción
// La respuesta en velocidad la da otra función (que es pública)
void ISRcalculaDelta()
{
  if (digitalRead(PinEntrada) != estadoSensor && micros() - usAnterior > 5000)
  {
    estadoSensor=!estadoSensor;       // Cambió el estado del sensor
    usDelta=(micros()-usAnterior);    // Calcula delta (sin filtro promediador)
    contadorISR++;                    // Agrega cuenta
    usAnterior=micros();           // Actualiza
    // usDelta --> Almacena cuanto pasó entre cada interrupción en microsegundos.
    // usAnterior --> Almacena el último tiempo de la interrupción en microsegundos.
  }
}

/******************************************************************************
* Funciones privadas
******************************************************************************/

// Configura la interrupción
void ConfigurarVelocidad(byte PIN)
{
  PinEntrada = PIN;             // Almacenamos el pin que debo leer 
  pinMode(PinEntrada, INPUT);   // Lo seteo como entrada digital
  attachInterrupt(digitalPinToInterrupt(PinEntrada),ISRcalculaDelta,CHANGE); 
              // Configuro la función a llamar en cada interrupción
  estadoSensor = digitalRead(PinEntrada);  
              // Muestro el estado inicial!!!
              // Verifico incialmente en qué estado está el sensor y lo guardo en la variable.
}

// Devuelve la velocidad en RPM según el tiempo entre cambios de estado
int velocidadRPM()
{
  int med = 0;
  if (usAnterior==0) {
    // Aún no hubo ninguna interrupción
    // Supongo una velocidad de 0 RPM
    med = 0;
    
  } else if ((micros()-usAnterior)>usDelta) {
    // micros()--> es el tiempo que paso desde que se inicio el programa en microsegundos
    // usAnterior --> el delta de la ultima interrupcion
    // usDelta --> el tiempo q paso entre la interrupcion anterior y la actual
    // Quiere decir que la velocidad real es menor a la que indica msDelta
    // (Esto puede ocurrir porque msDelta sólo se actualiza cuando ocurre la interrupción)
    med = (15000000/(micros()-usAnterior));
  } else {
    // Calculo con msDelta :-) 
    med = (15000000/usDelta);
    //3000RPM * 4 VUELTAS = 12000
    //12000/60= 200
    //1/200 = 0.005SEG
  }

  // Ya tengo un valor med: 
  // Puedo limitar y/o promediar su valor.
  med = min(med, 1.2*MAX_CAMBIOS_RPM);
  // med = 0.7*velocidadAnterior + 0.3*med;
  // velocidadAnterior = med;

  // Listo!!!
  return med;
}

/******************************************************************************
* FIN DE ARCHIVO velocidad.cpp
******************************************************************************/
