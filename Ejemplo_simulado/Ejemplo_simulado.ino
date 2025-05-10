/******************************************************************************
* CONTROL 
*******************************************************************************
*
*
******************************************************************************/

#include <math.h>
#include "pid_sca.h"

#define TIEMPO_MUESTREO    500
#define MODELO_RESISTENCIA 1
#define MODELO_CAPACIDAD   2
#define KP                 1
#define Ti                 10
#define TD                 0
#define SALIDA_MIN         0
#define SALIDA_MAX         10
#define LIMITAR_SALIDA     true
#define COMPENSAR_INTEGRAL true

unsigned long TiempoInicial;
unsigned long TiempoAnterior;
controlPID Corriente(KP, Ti, TD);
float VoltajeObjetivo      = 10;
float VoltajeSimulado      = 0;
float accionControl        = 0;
float accionInt            = 0;
float accionProp           = 0;
float compensacion         = 0;
int calculoActuador        = 0;

//*****************************************************************************

void setup() {
  
  // Limito la salida del actuador y seteo sus valores min y maximos
  Corriente.LimitarSalida( LIMITAR_SALIDA, SALIDA_MIN, SALIDA_MAX );
  Corriente.CompensarIntegral( COMPENSAR_INTEGRAL );
  
  // Presentación inicial de información
  Serial.begin(9600);
  Serial.println( "Tiempo \tObj \tMed \tCtrl \tProp \tInt \tComp");
  TiempoInicial = millis();
  TiempoAnterior = TiempoInicial;
  mostrar();

}

//*****************************************************************************

void loop() {

  do {
    
    // Acá se pueden poner otras acciones
    
  } while (millis() < (TiempoAnterior + TIEMPO_MUESTREO));
  TiempoAnterior = TiempoAnterior + TIEMPO_MUESTREO;

  // Pasado el intervalo deltaT, mido, calculo control y actúo.

  // 1) LEEMOS MODELO ---------------------------------------------------------
  VoltajeSimulado = sistemaSimulado( accionControl ); 
  
  // 2) CALCULO ACCION DE CONTROL ---------------------------------------------
  accionControl = Corriente.Controlar( VoltajeObjetivo - VoltajeSimulado ); 
 
  // 3) ACTUAR ----------------------------------------------------------------
  VoltajeSimulado = sistemaSimulado(accionControl); 
     // esto en realidad no hace nada porque es un sistema simulado
    
  // 4) MOSTRAR VALORES -------------------------------------------------------
  accionInt = Corriente.ObtenerIntegral();
  accionProp = Corriente.ObtenerProporcional();
  compensacion = Corriente.ObtenerCompensacion();
  
  // Enviar información:
  mostrar();
}

//*****************************************************************************

void mostrar()
{
  Serial.print(TiempoAnterior-TiempoInicial);
  Serial.print("\t");
  Serial.print(VoltajeObjetivo);
  Serial.print("\t");
  Serial.print(VoltajeSimulado);
  Serial.print("\t");
  Serial.print(accionControl);
  Serial.print("\t");
  Serial.print(accionProp);
  Serial.print("\t");
  Serial.print(accionInt);
  Serial.print("\t");
  Serial.println(compensacion);
}

//*****************************************************************************

float sistemaSimulado (float Corriente)
{
  // Valores del modelo del sistema
  const float Resistencia = MODELO_RESISTENCIA;
  const float Capacidad   = MODELO_CAPACIDAD;
  const float Tau         = Resistencia * Capacidad;

  // Variables almacenadas (estáticas)
  static float Voltaje = 0;
  static unsigned long T_Anterior = millis();

  // Variables locales dinñamicas
  unsigned long T_Actual = millis();
  unsigned long T_Delta = (T_Actual - T_Anterior) ;

  // Cambio de variabes de estado
  float VF = Corriente * Resistencia;
  Voltaje = (Voltaje-VF) * exp(-1.0*T_Delta/1000/Tau) + VF;
  T_Anterior = T_Actual;

  // Devolvemos estado actual
  return Voltaje;
}

//*****************************************************************************
