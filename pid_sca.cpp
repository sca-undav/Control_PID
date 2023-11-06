/**************************************************************************************************
* Control PID
***************************************************************************************************
* Sistemas de Control Automático (SCA)
* Universidad Nacional de Avellaneda (UNDAV)
*
* Archivo:    pid_sca.cpp
* Versión:    2.1. 
* Fecha:      Noviembre 2023
* Novedades:  Reorganización de funciones en .h y .cpp
* Versión Anterior: 2.0, diciembre 2021.
**************************************************************************************************/

#include "pid_sca.h"
#include "Arduino.h"

/*************************************************************************************************/

controlPID::controlPID(float KP, float TI, float TD)                   
// Constructor: incluye configuración inicial del PID y valores predeterminados.
{  // Kp puede ser negativo (esto último podría servir para controlar una planta cuya salida 
   //                        tienda a bajar cuando aumente la señal de control. Ej.: heladera.) 
   // Si Ti=0, el PID no lo tomará en cuenta
   // Si Td=0, el PID no lo tomará en cuenta
   // Inicializa integración e impone false en límites y compensación.
   ConfigurarPID(KP, TI, TD);
   LimitaSalida=false;
   CompensaIntegral=false;
}
//-------------------------------------------------------------------------------------------------

void controlPID::ConfigurarPID(float KP, float TI, float TD)
// Configura las constantes básicas del control PID 
// Sirve para cambiar configuración inicial, sin modificar límites y banderas.
{  // Kp puede ser negativo (esto último podría servir para controlar una planta cuya salida 
   //                        tienda a bajar cuando aumente la señal de control. Ej.: heladera.) 
   // Si Ti=0, el PID no lo tomará en cuenta
   // Si Td=0, el PID no lo tomará en cuenta
   Kp=KP;
   Ti=TI;
   Td=TD;
   // Resetea valores de integración.
   TiempoAnterior=0;
   ErrorAnterior=0;
   CompensacionAnterior=0;
   Integral=0;
}
//-------------------------------------------------------------------------------------------------

bool controlPID::LimitarSalida()
// Devuelve el valor de la variable privada LimitaSalida
// que indica si nuestro PID está configurado para limitar su salida.
{  return LimitaSalida;
}
//-------------------------------------------------------------------------------------------------

bool controlPID::LimitarSalida(bool RESPUESTA)
// Configura si limitará la salida...
// No permite activar límites si antes no fueron establecidos.
{  LimitaSalida=RESPUESTA;
   if (SalidaMax==0 && SalidaMin==0) {
     // ...no voy a limitar porque no tengo límites definidos.
     LimitaSalida=false;
   }
   return LimitaSalida;
}
//-------------------------------------------------------------------------------------------------

bool controlPID::LimitarSalida(bool RESPUESTA, float SMIN, float SMAX)
// Configura si limitará la salida entre SMAX y SMIN.
// Se puede establecer los límites pero no activarlos aún.
// No activa con SMIN=SMAX.
// No activa si SMIN>SMAX.
{  SalidaMax=SMAX;
   SalidaMin=SMIN;
   LimitaSalida=RESPUESTA;
   // La forma de desactivar este límite es:
   // 1) Volviendo a llamar esta función con RESPUESTA=false
   // 2) Llamando a LimitarSalida(false)
   // También se pueden poner límites muy grandes.
   if (SalidaMax==SalidaMin) {
     // ...no voy a limitar porque no tengo límites definidos.
     LimitaSalida=false;
   }
   if (SalidaMin>SalidaMax) {
     // ...no voy a limitar porque están mal configurados.
     LimitaSalida=false;
   }
   return LimitaSalida;
}
//-------------------------------------------------------------------------------------------------

bool controlPID::CompensarIntegral()
// Devuelve el valor de CompensaIntegral.
{  return CompensaIntegral;
}
//-------------------------------------------------------------------------------------------------

bool controlPID::CompensarIntegral(bool RESPUESTA)
// Establece si debo compensar la integración cuando la salida está saturada.
// Deben haberse preestablecido los límites de salida.
{  
   CompensaIntegral = RESPUESTA;
   if (!LimitaSalida) CompensaIntegral=false;
   return CompensaIntegral;
}
//-------------------------------------------------------------------------------------------------

float controlPID::Controlar(float ERROR)
// Calcula Salida en función de la señal error y los parámetros del PID
{  unsigned long TiempoActual = micros(); // Tomo tiempo actual para comparar con anterior
   
   // PROPORCIONAL --------------------------------------------------------------------------------
   Proporcional = Kp*ERROR;

   // DERIVATIVO ----------------------------------------------------------------------------------
   if (TiempoAnterior>0 && Td!=0) {  
      // Dos condiciones para componente derivativa:
      // 1) Que no sea el primer cálculo y 2) Td seteado
      Derivativo = Kp*Td*(ERROR-ErrorAnterior)*MILLON / (TiempoActual-TiempoAnterior);
   } else { 
      Derivativo = 0;
   }

   // ¿Debo compensar? ----------------------------------------------------------------------------
   Salida = Proporcional + Integral + Derivativo;
   Compensacion = 0;
   if (LimitaSalida && CompensaIntegral) {
         // Si no hay límite de saturación a la salida, no hay nada que compensar...
         if (Salida > SalidaMax) {
            // Debo saturar la salida porque supera el máximo...
            Compensacion = Salida - SalidaMax;
         }
         if (Salida < SalidaMin) {
            // Debo saturar la salida porque está por debajo del mínimo...
            Compensacion = Salida - SalidaMin;
         }      
   }
   
   // INTEGRAL ------------------------------------------------------------------------------------
   if (TiempoAnterior>0 && Ti!=0) {
      // Cumplidas las condiciones para integrar: (Si Compensacion==0, no va a compensar nada...)
      Integral += ( Kp * (ERROR+ErrorAnterior) - (Compensacion+CompensacionAnterior) ) 
                  * (TiempoActual-TiempoAnterior) 
                  / (2*Ti*MILLON);
      if (LimitaSalida) {
        // Debo saturar la integral:
        // (se supone que esto sólo podría pasar si cambio los parámetros de integración)
        Integral = min(Integral, SalidaMax);
        Integral = max(Integral, SalidaMin);
      }
   }

   // Termina componente integral -----------------------------------------------------------------
   
   // Cáculo final completo: 
   Salida = Proporcional + Integral + Derivativo;

   if (LimitaSalida) {
      // Debo saturar la salida:
      // (se supone que esto sólo podría pasar si cambio los parámetros de integración)
      Salida = min(Salida, SalidaMax);
      Salida = max(Salida, SalidaMin);
   }
   TiempoAnterior = TiempoActual;
   ErrorAnterior = ERROR;
   CompensacionAnterior = Compensacion;
   return Salida;
   // Termina funcion PID -------------------------------------------------------------------------
}
//-------------------------------------------------------------------------------------------------

float controlPID::ObtenerIntegral()
{
   return Integral; 
}
//-------------------------------------------------------------------------------------------------

float controlPID::ObtenerProporcional()
{
   return Proporcional;
}
//-------------------------------------------------------------------------------------------------

float controlPID::ObtenerDerivativo()
{
   return Derivativo;
}
//-------------------------------------------------------------------------------------------------

float controlPID::ObtenerSalida()
{
   return Salida;
}
//-------------------------------------------------------------------------------------------------

float controlPID::ObtenerCompensacion()
{
   return Compensacion;
}
//-------------------------------------------------------------------------------------------------

void controlPID::Apagar()
{
   TiempoAnterior=0;
   ErrorAnterior=0;
   CompensacionAnterior=0;
   Integral=0;   
   Proporcional=0;
   Derivativo=0; 
}

/**************************************************************************************************
* FIN DE ARCHIVO pid_sca.cpp
**************************************************************************************************/