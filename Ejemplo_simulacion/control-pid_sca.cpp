/**************************************************************************************************
* Control PID - SCA UNDAV
***************************************************************************************************
* Archivo:    control-pid_sca.cpp
* Versión:    3.0 
* Fecha:      mayo 2025
* Versión Anterior: 2.1, noviembre 2023.
**************************************************************************************************/

#include "Arduino.h"
#include "control-pid_sca.h"

const float MILLON = 1e6;  // Constante para convertir micros() a segundos.

/**************************************************************************************************
* Funciones públicas
**************************************************************************************************/

controlPID::controlPID(uint8_t PIN_SALIDA)                   
{
   PinSalida = PIN_SALIDA;             // Guarda pin de salida y configura si corresponde
   if (PinSalida>0) {
      pinMode(PinSalida, OUTPUT);
      analogWrite(PinSalida, 0);
   }
   Admin = {};                         // Valores de administración reseteados.
   Configuracion = {};                 // Configuración reseteada.
   Configuracion.Kp = 1;               // Valor predeterminado (resto dejamos en 0).
   Configurar(&Configuracion);          // Innecesario pero conveniente.
}

//-------------------------------------------------------------------------------------------------

void controlPID::Configurar(pid_config_s * CONFIG)
// Configura todos los parámetros.
// Kp puede ser negativo.
// Si Ti=0, el PID no lo tomará en cuenta.
// Si Td=0, el PID no lo tomará en cuenta.
{  
   // Cargamos configuracion:
   Configuracion.Objetivo          = CONFIG->Objetivo;
   Configuracion.Kp                = CONFIG->Kp;
   Configuracion.Ti                = CONFIG->Ti;
   Configuracion.Td                = CONFIG->Td;
   Configuracion.LimiteSuperior    = CONFIG->LimiteSuperior;
   Configuracion.LimiteInferior    = CONFIG->LimiteInferior;
   Configuracion.CompensarIntegral = CONFIG->CompensarIntegral;

   // Verificaciones:
   if (Configuracion.LimiteSuperior < Configuracion.LimiteInferior) {
      float SW = Configuracion.LimiteSuperior;
      Configuracion.LimiteSuperior = Configuracion.LimiteInferior;
      Configuracion.LimiteInferior = SW;
      // Corregimos CONFIG
      CONFIG->LimiteSuperior = Configuracion.LimiteSuperior;
      CONFIG->LimiteInferior = Configuracion.LimiteInferior;
   }
   if ( Configuracion.LimiteSuperior == Configuracion.LimiteInferior) {
      LimitarSalida = false;
   } else {
      LimitarSalida = true;
   }
   CompensarIntegral(Configuracion.CompensarIntegral);
   // Corregimos CONFIG si CompensarIntegral fue modificado:
   CONFIG->CompensarIntegral = Configuracion.CompensarIntegral;  

   // Resetea valores de integración (aunque mantiene ComponenteIntegral)
   TiempoAnterior       = 0;
   ErrorAnterior        = 0;
   CompensacionAnterior = 0;

   // Apago salida (si está activada)
   if (PinSalida>0) {
      analogWrite(PinSalida, 0 );
   }
}

//-------------------------------------------------------------------------------------------------

bool controlPID::CompensarIntegral()
// Devuelve el valor de CompensaIntegral.
{  
   return Configuracion.CompensarIntegral;
}
//-------------------------------------------------------------------------------------------------

bool controlPID::CompensarIntegral(bool COMPENSAR)
// Establece si debo compensar la integración cuando la salida está saturada.
// Deben haberse preestablecido los límites de salida.
{  
   if (true==COMPENSAR) {
      // ¿Se podrá compensar?
      if ( false==LimitarSalida ) {
         // No puedo compensar porque no hay límites definidos correctamente
         Configuracion.CompensarIntegral = false;
      } else {
         // Todo OK para compensar
         Configuracion.CompensarIntegral = true;
      }
   } else {
      // No hay condicionante para no compensar
      Configuracion.CompensarIntegral = false;
   }
   return Configuracion.CompensarIntegral;
}

//-------------------------------------------------------------------------------------------------

float controlPID::Controlar(float MEDICION)
// Calcula Salida en función de la MEDICION de entrada y la configuración del PID
{
   TiempoActual = micros();
   Admin.UltimaMedicion = MEDICION;
   Error = Configuracion.Objetivo - MEDICION;
   
   // PROPORCIONAL --------------------------------------------------------------------------------
   Admin.ComponenteProporcional = Configuracion.Kp * Error;

   // DERIVATIVO ----------------------------------------------------------------------------------
   if (TiempoAnterior>0 && Configuracion.Td!=0) {  
      // Dos condiciones para componente derivativa:
      // 1) Que no sea el primer cálculo y 2) Td seteado
      Admin.ComponenteDerivativo = Configuracion.Kp * Configuracion.Td
                                 * (Error-ErrorAnterior) * MILLON 
                                 / (TiempoActual-TiempoAnterior);
   } else { 
      Admin.ComponenteDerivativo = 0;
   }

   // ¿Debo compensar? ----------------------------------------------------------------------------
   Admin.Salida = Admin.ComponenteProporcional 
                + Admin.ComponenteIntegral 
                + Admin.ComponenteDerivativo;
   Admin.Compensacion = 0;
   if (Configuracion.CompensarIntegral) {
      // Si esto es true es porque había límites definidos correctamente
      if (Admin.Salida > Configuracion.LimiteSuperior) {
         // Debo compensar porque supera el máximo...
         Admin.Compensacion = Admin.Salida - Configuracion.LimiteSuperior;
      }
      if (Admin.Salida < Configuracion.LimiteInferior) {
         // Debo compensar porque está por debajo del mínimo...
         Admin.Compensacion = Admin.Salida - Configuracion.LimiteInferior;
      }      
   }
   
   // INTEGRAL ------------------------------------------------------------------------------------
   if (TiempoAnterior>0 && Configuracion.Ti!=0) {
      // Cumplidas las condiciones para integrar: (Si Compensacion==0, no va a compensar nada...)
      Admin.ComponenteIntegral = Admin.ComponenteIntegral
                               + ( Configuracion.Kp * (Error+ErrorAnterior) - (Admin.Compensacion+CompensacionAnterior) ) 
                               * ( TiempoActual-TiempoAnterior ) 
                               / ( 2*Configuracion.Ti*MILLON );
      if ( true==LimitarSalida ) {
        // Debo saturar la integral: (se supone que esto sólo podría pasar si cambio los parámetros de integración)
        Admin.ComponenteIntegral = min(Admin.ComponenteIntegral, Configuracion.LimiteSuperior);
        Admin.ComponenteIntegral = max(Admin.ComponenteIntegral, Configuracion.LimiteInferior);
      }
   }

   // Termina componente integral -----------------------------------------------------------------
   
   // Cáculo final completo: 
   Admin.Salida = Admin.ComponenteProporcional 
                + Admin.ComponenteIntegral 
                + Admin.ComponenteDerivativo;
   if ( true==LimitarSalida ) {
      // Debo saturar la salida: (se supone que esto sólo podría pasar si cambio los parámetros de integración)
      Admin.Salida = min(Admin.Salida, Configuracion.LimiteSuperior);
      Admin.Salida = max(Admin.Salida, Configuracion.LimiteInferior);
   }
   
   // Accion de control (si PIN_SALIDA está definido) ---------------------------------------------
   if (PinSalida>0 && LimitarSalida ) {
      analogWrite(PinSalida, int ( (Admin.Salida - Configuracion.LimiteInferior) 
                                 / (Configuracion.LimiteSuperior - Configuracion.LimiteInferior) 
                                 * 1014 ) );
   }

   // Termina funcion PID -------------------------------------------------------------------------
   TiempoAnterior = TiempoActual;
   ErrorAnterior = Error;
   CompensacionAnterior = Admin.Compensacion;
   return Admin.Salida;
}

float controlPID::Controlar(float MEDICION, float OBJETIVO)
{
   Configuracion.Objetivo = OBJETIVO;
   return Controlar (MEDICION);
}

//-------------------------------------------------------------------------------------------------

void controlPID::Apagar()
{
   TiempoAnterior=0;
   ErrorAnterior=0;
   CompensacionAnterior=0;
   Admin.ComponenteIntegral=0;   
   Admin.ComponenteProporcional=0;
   Admin.ComponenteDerivativo=0;
   if (PinSalida>0) {
      analogWrite(PinSalida, 0 );
   } 
}

//-------------------------------------------------------------------------------------------------

void controlPID::Leer(pid_info_s * INFO)
{
   INFO->Salida                 = Admin.Salida;
   INFO->ComponenteProporcional = Admin.ComponenteProporcional;
   INFO->ComponenteIntegral     = Admin.ComponenteIntegral;
   INFO->ComponenteDerivativo   = Admin.ComponenteDerivativo;
   INFO->Compensacion           = Admin.Compensacion;
   INFO->UltimaMedicion         = Admin.UltimaMedicion;
   INFO->LimitarSalida          = Admin.LimitarSalida;
}

/**************************************************************************************************
* FIN DE ARCHIVO control-pid_sca.cpp
**************************************************************************************************/