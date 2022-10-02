# Detector-Sintomas-COVID-G7
Uso del ESP32CAM con los sensores Max30100 y termperatura por IR para medir ritmo cardiaco, tempertura y oxigenación, lo manda por MQTT y lo guarda en base de datos MySQL


Este ejercicio consta de cuatro etapas
*   Creae base de datos con su tabla de registros en MySQL
* Construir el circuito con ESP32CAM que adquiera los datos de los sensores y programarlo para enviarlo por MQTT
* Programar el Flow en Node-Red que maneje la información y la guarde en la base de datos de MySQL ubicada en equipo local
* Usar la base de datos de MySQL para graficar en el servidor Grafana y usarlas nuevas graficas para reemplazar las graficas de Node-red

Como base de circuito para el ESP32CAM se toman de referencia el que existe en el repositorio de Codigo IoT (PONER LA URL)

y para el uso del sensor Max30100 que pueda leer la oxigenación se usa el ejemplo [Optical SP02 Detection (SPK Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 19th, 2016](https://github.com/sparkfun/MAX30105_Breakouthttps://github.com/sparkfun/MAX30105_Breakout)

Para la base de datos se ocupa instalar MySQL.
* Actualizar sistema

      sudo apt-get update
* Instalar MySQL

      sudo apt install mysql-server
*  Actualizar sistema

       sudo apt-get update
* Entrar a MySQL

      sudo mysql
* Crear Base de datos con nombre "DetctorSintomas"

      create databases DetectorSintomas;

* Ver las bases de datos existentes

      show databases;

* Usar una base de datos

      use DetectorSintomas;
* Ver las tablas existentes en la base de datos

      show tables;
* Crear una tabla dentro de la base de datos usada, con los pares de Encabezdo de columna y el tipo de dato de la columna (Nombre_Columna Tipo_de_ dato, Nombre_Columna Tipo_de_ dato, ... )

      create table Registro (id INT(6) UNSIGNED AUTO_INCREMENT PRIMARY KEY, fecha TIMESTAMP DEFAULT CURRENT_TIMESTAMP, nombre CHAR (248) NOT NULL, correo CHAR (248) NOT NULL, temp FLOAT(4,2) NOT NULL, bpm INT(1) UNSIGNED NOT NULL, sp02 INT(1) UNSIGNED NOT NULL, protodiagnostico CHAR (248) NOT NULL);

* Ver el tipo de datos de cada columna de la tabla 

      Describe table Registro;

* Ver el contenido de los registros de la tabla, se muestra cada valor

      selec *from Registro;
* Importante no olvidar el uso del ";" al final de la instrucción en MySQL.

* Crear usuario con privilegios 
      
      Ceate.user'Nombre'@'localhost' Identified by "password";
      mysql GRANT ALL PRIVILEGES ON;

EN Node-Red se ocupan los nodos de MySQL para enviarle datos a la base de datos




