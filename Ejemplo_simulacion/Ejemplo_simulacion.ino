/******************************************************************************
* CONTROL 
*******************************************************************************
*
*
******************************************************************************/

#include <math.h>
#include "control-pid_sca.h"

#define TIEMPO_MUESTREO    500
#define MODELO_RESISTENCIA 1
#define MODELO_CAPACIDAD   2
#define OBJETIVO           10
#define K_PROP             5
#define T_INT              4
#define T_DER              0
#define SALIDA_MIN         0
#define SALIDA_MAX         20
#define COMPENSAR_INTEGRAL_AFIRMATIVO true

controlPID    Carga (PID_SIN_SALIDA);
pid_config_s  ConfiguracionPID = {0};
pid_info_s    InformePID       = {0};
float         VoltajeSimulado  = 0;
float         AccionControl    = 0;
unsigned long TiempoInicial    = 0;
unsigned long TiempoAnterior   = 0;

//*****************************************************************************

void setup() {
  
  // Configura PID
  ConfiguracionPID.Objetivo = OBJETIVO;
  ConfiguracionPID.Kp       = K_PROP;
  ConfiguracionPID.Ti       = T_INT;
  ConfiguracionPID.Td       = T_DER;
  ConfiguracionPID.LimiteSuperior    = SALIDA_MAX;
  ConfiguracionPID.LimiteInferior    = SALIDA_MIN; 
  ConfiguracionPID.CompensarIntegral = COMPENSAR_INTEGRAL_AFIRMATIVO;
  Carga.Configurar( &ConfiguracionPID );
  
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

  // Pasado el intervalo TIEMPO_MUESTREO: mido, calculo control y actúo.

  // 1) LEEMOS MODELO ---------------------------------------------------------
  VoltajeSimulado = sistemaSimulado( AccionControl ); 
  
  // 2) CALCULO ACCION DE CONTROL ---------------------------------------------
  AccionControl = Carga.Controlar( VoltajeSimulado ); 
 
  // 3) ACTUAR ----------------------------------------------------------------
  VoltajeSimulado = sistemaSimulado( AccionControl ); 
  // esto en realidad no hace nada porque es un sistema simulado
    
  // 4) MOSTRAR VALORES -------------------------------------------------------
  Carga.Leer( &InformePID );
  mostrar();

}

//*****************************************************************************

void mostrar()
{
  Serial.print(TiempoAnterior-TiempoInicial);
  Serial.print("\t");
  Serial.print(ConfiguracionPID.Objetivo);
  Serial.print("\t");
  Serial.print(VoltajeSimulado);
  Serial.print("\t");
  Serial.print(AccionControl);
  Serial.print("\t");
  Serial.print(InformePID.ComponenteProporcional);
  Serial.print("\t");
  Serial.print(InformePID.ComponenteIntegral);
  Serial.print("\t");
  Serial.println(InformePID.Compensacion);
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
