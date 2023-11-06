/******************************************************************************
* CONTROL PI DE UN ELECTROVENTILADOR
*******************************************************************************
*
* El objetivo es controlar la velocidad de un electroventilador de un auto.
* Para medir su velocidad, el ventilador tiene dos marcas blancas y dos negras 
* alrededor de su circunferencia.
*
* Curso de Sistemas de Control Automático, UNDAV.
* 2º cuatrimestre 2022.
*
* Lenguaje C++ con Arduino.
*
******************************************************************************/

#include "pid_sca.h"
#include "velocidad.h"

const int pinActuador = 5;         // Pin hacia el transistor. Capacidad de PWM
const int pinSensor = 2;           // Pin del sensor
const int pinPotenciometro = A2;   // pin del potenciometro
const int pinLedIntegrado = 13;    // Pin del led que trae integrada la placa
const int pinBotonParada = 7;      // Pin del boton de parada de emergencia
const int deltaT = 50;             // Intervalo entre muestras, en milisegundos

bool estadoBotonParada = false;
long tiempoAnterior;

int objetivoRPM = 0;
int electroRPM = 0;

float Kp = 0.08; // Kp se multiplica por 12 para cambiar la escala a voltios
float Ti = 5;
float Td = 0;
float SalidaMin= 0;
float SalidaMax= 12;
const bool limitarSalidaTrue = true;
const bool CompensaIntegralTrue = true; // Denota si se esta compensando 
                                        // la integral para el antienrole

controlPID controlador(Kp, Ti, Td);
float accionControl = 0;
float accionInt = 0;
float accionProp = 0;
float compensacion = 0;
int calculoActuador=0;

//*****************************************************************************

void setup() {
  
  // Configuración de pines
  ConfigurarVelocidad(pinSensor);           
  pinMode(pinPotenciometro, INPUT);
  pinMode(pinActuador, OUTPUT);
  pinMode(pinBotonParada, INPUT);
  pinMode(pinLedIntegrado, OUTPUT);
  digitalWrite(pinLedIntegrado, LOW); // Fuerzo led apagado.

  // Presentación inicial de información
  tiempoAnterior = millis();
  Serial.begin(57600);
  Serial.println( "Tiempo \tObj \tMed \tCtrl \tProp \tInt \tComp");
  mostrar( tiempoAnterior, objetivoRPM, electroRPM, 
           accionControl, accionProp, accionInt, compensacion ); 
   
  // Limito la salida del actuador y seteo sus valores min y maximos
  controlador.LimitarSalida( limitarSalidaTrue, SalidaMin, SalidaMax );
  controlador.CompensarIntegral( CompensaIntegralTrue );

}

//*****************************************************************************

void loop() {

  do {
    estadoBotonParada = digitalRead(pinBotonParada);

    // si se presionó el boton de emergencia --> paro el sistema
    if (estadoBotonParada)
    {
      Serial.print("Se ha accionado la parada de Emergencia");
      paramosSistema();
    }
  } while (millis() < (tiempoAnterior + deltaT));
  tiempoAnterior = tiempoAnterior + deltaT; // Actualiza el tiempo del loop

  // Pasado el intervalo deltaT, mido, calculo control y actúo.

  // 1) LECTURAS --------------------------------------------------------------
  // Leemos las RPM objetivo desde el potenciometro
  objetivoRPM = lecturaPotenciometro(pinPotenciometro);
  
  // Leemos la velocidad del electroventilador
  electroRPM = velocidadRPM(); 
  
  // 2) CALCULO DE CONTROL ----------------------------------------------------
  // Paso la diferencia entre RPM leída y objetivo
  // Ajusta la tensión del ventilador en base a los RPM
  accionControl = controlador.Controlar( objetivoRPM - electroRPM ); 
 
  // 3) ACTUAR ----------------------------------------------------------------
  // Cambio la accion de control para que dé un ciclo de trabajo entre 0 y 255
  calculoActuador = accionControl * 255/12;
  analogWrite( pinActuador, calculoActuador );
    
  // 4) MOSTRAR VALORES -------------------------------------------------------
  accionInt = controlador.ObtenerIntegral();
  accionProp = controlador.ObtenerProporcional();
  compensacion = controlador.ObtenerCompensacion();
  
  // Enviar información:
  mostrar( tiempoAnterior, objetivoRPM, electroRPM, 
           accionControl, accionProp, accionInt, compensacion );

}

//*****************************************************************************

void paramosSistema()
{
  // 1) parar todo
  //controlador.Apagar();
  analogWrite(pinActuador,0);
  // 2) enviar info --> parpadea led
  // 3) loop infinito
  Serial.println("Boton de parada accionado.");
  do
  {

    digitalWrite(pinLedIntegrado, LOW);
    delay(1000);
    digitalWrite(pinLedIntegrado, HIGH);
    delay(500);
  } while (true);
}

//*****************************************************************************

void mostrar( unsigned long tiempo, unsigned int rpm, unsigned int measurerpm, 
              float accionControl, float accionProp, float accionInt, 
              float compensacion)
{
  Serial.print(tiempo);
  Serial.print("\t");
  Serial.print(rpm);
  Serial.print("\t");
  Serial.print(measurerpm);
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

int lecturaPotenciometro(int pinPotenciometro)
{
	// Lectura del potenciometro:
  int valorPotenciometro = analogRead(pinPotenciometro);

  // Mapeo de valor entre 0 y maxPRM:
  int objetivoRPM = map(valorPotenciometro, 0, 1023, 0, MAX_CAMBIOS_RPM);
  
  // Redondeo y truncamiento
  objetivoRPM = (( objetivoRPM+50) /250)*250;
  if (objetivoRPM < 150) {
    objetivoRPM = 0;
  }

  return objetivoRPM;
}

//*****************************************************************************
