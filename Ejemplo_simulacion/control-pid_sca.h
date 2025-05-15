/**************************************************************************************************
* Control PID - SCA UNDAV
***************************************************************************************************
* Archivo:    control-pid_sca.h
* Breve:      Módulo de control PID desarrollado para Sistemas de Control Automático (SCA),
              Universidad Nacional de Avellaneda (UNDAV). Utiliza POO en C++ para Arduino.
* Versión:    3.0. 
* Fecha:      mayo 2025
* Novedades:  - Redefinición de funciones y parámetros.
*             - Constructor posee sólo pin de salida (si se utiliza).
*             - La configuracion se hace mediante un struct específico.
*             - La informacion de variables se pasa mediante puntero a una estructura.
*             - Version simplificada de configuración de compensación anti enrole.
* Versión Anterior: 2.1, noviembre 2023.
**************************************************************************************************/

#ifndef CONTROL_PID_SCA_H
#define CONTROL_PID_SCA_H

#define PID_SIN_SALIDA 0

struct pid_config_s {
   float Objetivo;            // Salida Objetivo del sistema.
   float Kp;                  // Constante de proporcionalidad (puede ser negativo)
   float Ti;                  // Tiempo de integración (si es 0, no integra), en segundos
   float Td;                  // Tiempo de derivación (si es 0 no deriva), en segundos
   float LimiteSuperior;      // Límite superior de la salida y de la integral
   float LimiteInferior;      // Límite inferior de la salida y de la integral
   bool  CompensarIntegral;   // true si se desea compensar la integral ante enrole
};

struct pid_info_s {
   float Salida;                 // La señal de control que va al acuador
                                 // o potencia de salida (sin asignar unidades)
   float ComponenteProporcional; // Componente proporcional de la salida 
                                 // (sin asignar unidades) 
   float ComponenteIntegral;     // Componente integral 
   float ComponenteDerivativo;   // Componente derivativo 
   float Compensacion;           // Contiene la compensacion resultante ante saturación
   float UltimaMedicion;
   bool  LimitarSalida;
};

class controlPID                          // Clase para control PID
{  
   private:
   pid_config_s  Configuracion;           // Parámetros configurados.    
   pid_info_s    Admin;                   // Variables de administración del control PID
   uint8_t       PinSalida;
   bool          LimitarSalida;
   unsigned long TiempoActual;          
   unsigned long TiempoAnterior;          // Tiempo de la medición anterior utilizando micros
   float         Error;
   float         ErrorAnterior;           // Señal de error anterior 
   float         CompensacionAnterior;    // Como usamos aproximación trapezoidal de la integral,
      
   public:
   controlPID(uint8_t PIN_SALIDA);        // Constructor con PIN de salida.
                                          // - PIN_SALIDA debe ser un PWM válido de Arduino.
                                          // - Si PIN_SALIDA = 0, no modifica nivel de PWM.
   void Configurar(pid_config_s *CONFIG); // Configura todos los parámetros.
   void Obtener(pid_config_s *CONFIG);    // Obtiene los parámetros configurados.
   bool CompensarIntegral(bool COMPENSAR); 
                                          // Activa o desactiva la compansación de integración 
                                          // e indica si está activado:
   bool CompensarIntegral();              // Indica si la compansación está activada.
   float Controlar(float MEDICION, float OBJETIVO);  
                                          // Calcula señal de control en función de la MEDICION y
                                          // el OBJETIVO. Configura el objetivo actual.
   float Controlar(float MEDICION);       // Calcula señal de control en función de la MEDICION.
                                          // Si Objetivo está configurado en 0, puede utilizarse 
                                          // como MEDICION-Objetivo = -ERROR
   void Apagar();                         // Apaga el PID manteniendo configuración.
   void Leer(pid_info_s * INFO);          // Lee la acción de control, componente proporcional, 
                                          // integral y otros datos de funcionamiento.
};

/*************************************************************************************************/

#endif // CONTROL_SINO_H

/******************* FIN DE ARCHIVO **************************************************************/