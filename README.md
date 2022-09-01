# Detector-Sintomas-COVID-G7
Uso del ESP32CAM con los sensores Max30100 y termperatura por IR para medir ritmo cardiaco, tempertura y oxigenación, lo manda por MQTT y lo guarda en base de datos MySQL


Este ejercicio consta de tres etapas
*   Creae base de datos con su tabla de registros en MySQL
* Construir el circuito con ESP32CAM que adquiera los datos de los sensores y programarlo para enviarlo por MQTT
* Programar el Flow en Node-Red que maneje la información y la guarde en la base de datos de MySQL ubicada en equipo local

Como base de circuito para el ESP32CAM se toman de referencia el que existe en el repositorio de Codigo IoT (PONER LA URL)

y para el uso del sensor Max30100 que pueda leer la oxigenación se usa el ejemplo [Optical SP02 Detection (SPK Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 19th, 2016](https://github.com/sparkfun/MAX30105_Breakouthttps://github.com/sparkfun/MAX30105_Breakout)

