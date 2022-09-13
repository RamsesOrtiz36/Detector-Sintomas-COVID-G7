/*
 * Envio de JSON por MQTT
 * por: Hugo Escalpelo
 * Fecha: 15 de noviembre de 2021
 * 
 * Este programa envía datos a por Internet a través del protocolo MQTT. Para poder
 * comprobar el funcionamiento de este programa, es necesario conectarse a un broker
 * y usar NodeRed para visualzar que la información se está recibiendo correctamente.
 * Este programa no requiere componentes adicionales.
 * 
 * Este programa está basado en "Conexión básica por MQTT del NodeMCU" y explica como
 * enviar strings que contengan JSON para enviar mas de una variable a la vez.
 * 
 * Modificado por RAmses Ortiz Castro para el ejercicio de DEtector de sintomas COVID 
 * Fecha: 02 septiembre 2022
 * Se agrega la lectura de los sensores de Ritmo cardiaco, oxigenación (MAX30100)y temperatura por IR (GY 90614) Por I2C
 * 
 * Se envian los valores de sensores por JSON mediante MQTT a Node-Red
 * 
 * ESP32 CAM      MAX30100      GY90614
 * 5 Vcc ---------VIN------------VIN
 * GND------------GND------------GND
 * 14-------------SCL------------SCL
 * 15-------------SDA------------SDA
 * 
 * 
 * Se toma en base el programa de enviar JSON por MQTT y tomando el Example8_SPO2 Arduino 1.8.19 para agregar el proceso de comunicación por I2C
 */

//Bibliotecas
#include <WiFi.h>  // Biblioteca para el control de WiFi
#include <PubSubClient.h> //Biblioteca para conexion MQTT
#include <Wire.h>         //Bilbioteca para comunicación I2C
#include "MAX30105.h"     //Biblioteca para el uso del sensor MAX3010x
#include "spo2_algorithm.h" //Biblioteca para determinar la oxigenación


//Datos de WiFi
const char* ssid = "INFINITUM1885_2.4";  // Aquí debes poner el nombre de tu red
const char* password = "7pxYYmS6EP";  // Aquí debes poner la contraseña de tu red

//Datos del broker MQTT
const char* mqtt_server = "192.168.1.65"; // Si estas en una red local, coloca la IP asignada, en caso contrario, coloca la IP publica
IPAddress server(192,168,1,65);

// Objeros
WiFiClient espClient; // Este objeto maneja los datos de conexion WiFi
PubSubClient client(espClient); // Este objeto maneja los datos de conexion al broker
MAX30105 particleSensor;    //Objeto para usarse con el sensor MAX30100

#define MAX_BRIGHTNESS 255
//Verifica que tipo de Micro conrolador se usa, para declarar el tamaño de los arreglos a usar, si son Atmega entonces si de 32 si no lo son entonces de 16 bits
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif


// Variables para enviar por MQTT
int ledPin = 4;  // Para indicar el estatus de conexión
int ledPin2 = 33; // Para mostrar mensajes recibidos
long timeNow, timeLast; // Variables de control de tiempo no bloqueante
int wait = 5000;  // Indica la espera cada 5 segundos para envío de mensajes MQTT

//variables para el sensor MAX30100
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

//byte pulseLED = 11; //Must be on PWM pin
//byte readLED = 13; //Blinks with each data read




// Inicialización del programa
void setup() {
  // Iniciar comunicación serial
  Serial.begin (115200);

  //Configura el uso de pines 
  pinMode (ledPin, OUTPUT);       // Para indicar el estatus de conexión
  pinMode (ledPin2, OUTPUT);      // Para mostrar mensajes recibidos
  digitalWrite (ledPin, HIGH);    // Para indicar el estatus de conexión activada
  digitalWrite (ledPin2, HIGH);   // Para mostrar mensajes recibidos activada

  //Mensaje de conexión a red WiFi por monitor serial
  Serial.println();               //salta linea en blanco en el monitor serial
  Serial.println();               //salta linea en blanco en el monitor serial
  Serial.print("Conectar a ");    //Mensaje en monitor serial
  Serial.println(ssid);           //indica en monitor serial el nombre de la red a la que se conecta
 
  WiFi.begin(ssid, password); // Esta es la función que realiza la conexión a WiFi 
 
  while (WiFi.status() != WL_CONNECTED) {   // Este bucle espera a que se realice la conexión
    digitalWrite (ledPin, HIGH);            //Activa LED para indicar que esta inicaiando el proceso a conexion de red WiFi
    delay(500);                             //dado que es de suma importancia esperar a la conexión, debe usarse espera bloqueante con "delay"
    digitalWrite (ledPin, LOW);             //Apaga el LED de conexion a red WiFi
    Serial.print(".");                      // Indicador de progreso escribiendo linea de puntos mientras espera a conectarse"........"
    delay (5);                              //Reintenta la conexion si no se realizo, regresando al "while"
  }
  
  // Cuando se haya logrado la conexión, el programa avanzará, por lo tanto, puede informarse lo siguiente
  Serial.println();
  Serial.println("WiFi conectado");
  Serial.println("Direccion IP: ");
  Serial.println(WiFi.localIP());

  // Si se logro la conexión, encender led
  if (WiFi.status () > 0){
  digitalWrite (ledPin, HIGH);
  }
  
  delay (1000);                           // Esta espera es solo una formalidad antes de iniciar la comunicación con el broker

  // Conexión con el broker MQTT
  client.setServer(server, 1883);         // Conectarse a la IP del broker en el puerto indicado
  client.setCallback(callback);           // Activar función de CallBack, permite recibir mensajes MQTT y ejecutar funciones a partir de ellos
  delay(1500);                            // Esta espera es preventiva, espera a la conexión para no perder información

  
 // Initialize sensor MAX30100 por I2C----------------------------------------------------------------------------------------------------------------------------------------
  if (!particleSensor.begin(Wire))                                            //, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed  Si no se conecta el sensor por particleSensor manda el mensaje de que no se encontró el sensor
  {
    Serial.println(F("MAX30100 was not found. Please check wiring/power."));  //Mensaje de que no se encontró el sensor y revisar el cable de alimentación del sensor
    while (1);                                                                // 
  }

  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));     // Si si se encontró el sensor manda mensaje de que se sujete al dedo con una liga y presionar cualquier tecla 
  while (Serial.available() == 0) ;                                                                     //wait until user presses a key, espera a que se presione cualquier tecla
  Serial.read();                                                                                        //empieza a leer el sensor

//Variables de la configuración de lectura del sensor con dos LEDs, uno Rojo (RED) y el otro Infra rojo (IR)
  byte ledBrightness = 60;      //Options: 0=Off to 255=50mA//salta linea en blanco en el monitor serial
  byte sampleAverage = 4;       //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;             //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;        //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;         //Options: 69, 118, 215, 411
  int adcRange = 4096;          //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);      //Configure sensor with these settings
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



timeLast = millis ();                   // Inicia el control de tiempo dando una marca de tiempo a "timeLast" hasta ese momento justo cuando se inicia la coneción por broker
  
}// fin del void setup ()

// Cuerpo del programa, bucle principal
void loop() {
  //Verificar siempre que haya conexión al broker
  if (!client.connected()) {                    
    reconnect();                                // En caso de que no haya conexión, ejecutar la función de reconexión, definida despues del void setup ()
  }                                             // fin del if (!client.connected())
  client.loop();                                // Esta función es muy importante, ejecuta de manera no bloqueante las funciones necesarias para la comunicación con el broker

//Leer el sensor MAX30100 100 muestras y las guarda en redBuffer e irBuffer e imprime su valor en monitor serial

  bufferLength = 100;                                       //buffer length of 100 stores 4 seconds of samples running at 25sps
                                                            
  for (byte i = 0 ; i < bufferLength ; i++)                 //read the first 100 samples, and determine the signal range
  {
    while (particleSensor.available() == false)             //do we have new data?
      particleSensor.check();                               //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();                            //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }
  
//Determinar el ritmo cardiaco y oxigenación
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


//Leer Sensor de Temperatura IR

  
  timeNow = millis();                           // Control de tiempo para esperas no bloqueantes
  if (timeNow - timeLast > wait) 
  {                                             // Manda un mensaje por MQTT cada cinco segundos
    timeLast = timeNow;                         // Actualización de seguimiento de tiempo

    //Se construye el string correspondiente al JSON que contiene 3 variables
    String json = "{\"id\":\"Ramses\",\"temp\":"+String(random(24, 37))+",\"hum\":"+String(random (59,89))+"}";
    Serial.println(json);                                   // Se imprime en monitor solo para poder visualizar que el string esta correctamente creado
    int str_len = json.length() + 1;                        //Se calcula la longitud del string
    char char_array[str_len];                               //Se crea un arreglo de caracteres de dicha longitud
    json.toCharArray(char_array, str_len);                  //Se convierte el string a char array    
    client.publish("codigoIoT/ejemplo/mqtt", char_array);   // Esta es la función que envía los datos por MQTT, especifica el tema y el valor
  }                                                         // fin del if (timeNow - timeLast > wait)
}// fin del void loop ()

// Funciones de usuario

// Esta función permite tomar acciones en caso de que se reciba un mensaje correspondiente a un tema al cual se hará una suscripción
void callback(char* topic, byte* message, unsigned int length) {

  // Indicar por serial que llegó un mensaje
  Serial.print("Llegó un mensaje en el tema: ");
  Serial.print(topic);

  // Concatenar los mensajes recibidos para conformarlos como una varialbe String
  String messageTemp; // Se declara la variable en la cual se generará el mensaje completo  
  for (int i = 0; i < length; i++) {  // Se imprime y concatena el mensaje
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  // Se comprueba que el mensaje se haya concatenado correctamente
  Serial.println();
  Serial.print ("Mensaje concatenado en una sola variable: ");
  Serial.println (messageTemp);

  // En esta parte puedes agregar las funciones que requieras para actuar segun lo necesites al recibir un mensaje MQTT

  // Ejemplo, en caso de recibir el mensaje true - false, se cambiará el estado del led soldado en la placa.
  // El NodeMCU está suscrito al tema esp/output
  if (String(topic) == "codigoIoT/ejemplo/mqttin") {  // En caso de recibirse mensaje en el tema codigoIoT/ejemplo/mqttin
    if(messageTemp == "true"){
      Serial.println("Led encendido");
      digitalWrite(ledPin2, LOW);
    }// fin del if (String(topic) == "codigoIoT/ejemplo/mqttin")
    else if(messageTemp == "false"){
      Serial.println("Led apagado");
      digitalWrite(ledPin2, HIGH);
    }// fin del else if(messageTemp == "false")
  }// fin del if (String(topic) == "codigoIoT/ejemplo/mqttin")
}// fin del void callback

// Función para reconectarse
void reconnect() {
  // Bucle hasta lograr conexión
  while (!client.connected()) { // Pregunta si hay conexión
    Serial.print("Tratando de contectarse...");
    // Intentar reconexión
    if (client.connect("ESP8266Client")) { //Pregunta por el resultado del intento de conexión
      Serial.println("Conectado");
      client.subscribe("codigoIoT/ejemplo/mqttin"); // Esta función realiza la suscripción al tema
    }// fin del  if (client.connect("ESP8266Client"))
    else {  //en caso de que la conexión no se logre
      Serial.print("Conexion fallida, Error rc=");
      Serial.print(client.state()); // Muestra el codigo de error
      Serial.println(" Volviendo a intentar en 5 segundos");
      // Espera de 5 segundos bloqueante
      delay(5000);
      Serial.println (client.connected ()); // Muestra estatus de conexión
    }// fin del else
  }// fin del bucle while (!client.connected())
}// fin de void reconnect(
