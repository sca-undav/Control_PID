/******************************************************************************
* Velocidad de cambios de estado
*******************************************************************************
* Archivo: velocidad.h
* Resumen: El módulo reúne algunas funciones para calcular la velocidad
*          con que un pin cambia de estado.
* Autor:   Guillermo Caporaletti <gfcaporaletti@undav.edu.ar>
* Fecha:   Abril de 2023
******************************************************************************/

/******************************************************************************
* Librería y módulos incluidas
******************************************************************************/

#include "arduino.h"

/******************************************************************************
* Constantes definidas
******************************************************************************/

#define MAX_CAMBIOS_RPM       3000                // Máxima cantidad de cambios de estado por minuto, físicamente posible
                                                  // (se recomienda poner un 50% más que el real).
#define MAX_CAMBIOS_PS        MAX_CAMBIOS_RPM/60  // Máxima cantidad de cambios de estado por segundo, físicamente posible.
#define CAMBIOS_P_VUELTA      4                   // Cambios de estado producidos en cada vuelta.
#define MAX_US_ENTRE_CAMBIOS  1000000 / ( MAX_CAMBIOS_PS * CAMBIOS_P_VUELTA )  
                                                  // Máximo tiempo posible entre cambios de estado.

/******************************************************************************
* Funciones públicas
******************************************************************************/

void ConfigurarVelocidad(byte PIN);               // Configura el pin a leer e inicializa variables
int velocidadRPM();                               // Mide la velocidad en función de las interrupciones que haya habido

/******************************************************************************
* FIN DE ARCHIVO velocidad.h
******************************************************************************/
