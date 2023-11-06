/**************************************************************************************************
* Control PID
***************************************************************************************************
* Sistemas de Control Automático (SCA)
* Universidad Nacional de Avellaneda (UNDAV)
*
* Archivo:    pid_sca.h
* Versión:    2.1. 
* Fecha:      Noviembre 2023
* Novedades:  Reorganización de funciones en .h y .cpp
* Versión Anterior: 2.0, diciembre 2021.
**************************************************************************************************/

class controlPID            // Objeto para control Proporcional-Integral-Derivativo (PID)
{  private:
   
   float Salida;                  // La señal de control que va al acuador
                                  // o potencia de salida (sin asignar unidades)
   float Proporcional;            // Componente proporcional de la salida 
                                  // (sin asignar unidades)
   float Integral;                // Componente integral 
   float Derivativo;              // Componente derivativa
   float Compensacion;            // Contiene la compensacion resultante ante saturación
   float CompensacionAnterior;    // Como usamos aproximación trapezoidal de la integral,
                                  // necesitamos conservar el valor anterior.
   float Kp;                      // Constante proporcional (sin asignar unidades)
   float Ti;                      // Tiempo de integración (en segundos)
   float Td;                      // Tiempo para la componente derivativa (en segundos)
   unsigned long TiempoAnterior;  // Tiempo de la medición anterior utilizando micros
   float ErrorAnterior;           // Señal de error anterior 
   bool LimitaSalida;             // Indica si establecimos límites superior e inferior
   bool CompensaIntegral;         // Indica si establecimos la compensacion de integral
   float SalidaMax;               // Límite superior de la salida (y de la integral)
   float SalidaMin;               // Límite inferior de la salida
   const float MILLON = 1e6;      // Constante para convertir micros() a segundos.
      
   public:

   // Constructor con lo mínimo:
   controlPID( float KP,          // KP: Constante de proporcionalidad (puede ser negativo)
               float TI,          // TI: Tiempo de integración (si es 0, no integra)
               float TD );        // TD: Tiempo de derivación (si es 0 no deriva)

   // Para cambiar configuración inicial:
   void ConfigurarPID(float KP, float TI, float TD);  //Mismos parámetros que constructor.

   // Configura los límites de salida e indica si están activados:
   bool LimitarSalida(bool RESPUESTA, float SMIN, float SMAX);  

   // Activa o desactiva los límites de salida e indica si el límite de salida está activado:
   bool LimitarSalida(bool RESPUESTA);  // No permite activar límites si antes no fueron 
                                        // establecidos.

   // Indica si el límite de salida está activado:
   bool LimitarSalida();                          

   // Activa o desactiva la compansación de integración e indica si está activado:
   bool CompensarIntegral(bool RESPUESTA);        

   // Me indica si la compansación está activada:
   bool CompensarIntegral();                     

   // Calcula señal de control (salida) en función del error:
   float Controlar(float ERROR);                        

   // Apaga el PID manteniendo configuración:
   void Apagar();                 // No se modifican los valores de KP, TI y TD.
                                  // Tampoco los límites pre establecidos.

   float ObtenerIntegral(); 
   float ObtenerProporcional(); 
   float ObtenerDerivativo(); 
   float ObtenerSalida();
   float ObtenerCompensacion();

};

/**************************************************************************************************
* FIN DE ARCHIVO pid_sca.h
**************************************************************************************************/