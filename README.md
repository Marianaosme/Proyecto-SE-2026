# Implementación De Un Sistema De Captura De Movimiento Para El Control De Una Mano Robótica 

## Integrantes 

- Mariana Ospina Mejía -- Technical Lead  
- Isaac Triana Melo -- Firmware Engineer
- Juan Carlos Martínez -- Hardware Integration Engineer
- Simon Eastman -- Verification & Testing Engineer

## Introdución y Problema 

Los sistemas embebidos son sistemas electrónicos diseñados para realizar funciones específicas dentro de un dispositivo más grande, integrando componentes de hardware y software que permiten ejecutar tareas de manera eficiente y automatizada. Estos sistemas se encuentran presentes en múltiples aplicaciones tecnológicas, como dispositivos médicos, sistemas de automatización industrial, vehículos inteligentes y equipos robóticos. 

En el campo de la robótica, los sistemas embebidos desempeñan un papel fundamental, ya que permiten procesar información proveniente de sensores y generar señales de control para actuadores, haciendo posible la interacción entre el entorno y el sistema robótico. 

En este proyecto se propone el desarrollo de un sistema de control para una mano robótica manejada a distancia. Para el desarrollo del prototipo, el sistema de control estara conectado directamente a la mano robótica mediante cables, con el fin de facilitar la transmisión de señales y simplificar la implementación del sistema. 

Para lograr el control de la mano robótica, se emplearán potenciómetros como sensores para detectar la flexión y extensión de los dedos, asi como un acelerómetro para medir la rotación de la muñeca. La información capturada por estos sensores será procesada por un sistema, encargado de interpretar las señales y enviar los comandos necesarios para replicar los movimientos en la mano robótica.  

De esta manera, el proyecto busca demostrar cómo los sistemas embebidos pueden integrarse con sensores y dispositivos robóticos, permitiendo desarrollar soluciones tecnológicas capaces de reproducir ciertos movimientos humanos y que, como proyección futura, puedan facilitar la manipulación remota de objetos. 


## Objetivos
### Objetivo  General 
Diseñar e implementar un sistema de control para una mano robótica, capaz de replicar ciertos movimientos de la mano humana, como la flexión y extensión de los dedos, y la rotación de la muñeca.  

### Objetivos Específicos
-  Diseñar la estructura mecánica de la mano robótica que permita la flexión y extensión de los dedos, la rotación de la muñeca y el posible agarre de diferentes objetos. 

- Desarrollar un sistema de control y comunicación que permita enviar y recibir señales para el manejo de la mano. 

- Realizar pruebas de funcionamiento para evaluar la capacidad de agarre, precisión de movimiento y respuesta del sistema ante diferentes comandos. 