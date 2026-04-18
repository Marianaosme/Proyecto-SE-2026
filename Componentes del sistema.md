# Componentes del sistema

A continuación, se presentan los componentes necesarios para la implementación del sistema, además de los diseños en CAD correspondientes a la mano y al guante de control respectivamente.  

### Estructura mecánica
| Componente | Observación |
|------------|-------------|
|Mano Robotica | Estructura principal del sistema, diseñada para simular la anatomía de una mano humana. Cuenta con un sistema de tendones accionados por servomotores que permiten la flexión y extensión de cada dedo. Las articulaciones están ensambladas mediante tornillos de 3 mm, tuercas y elementos como alambres para la transmisión del movimiento. |
|Guante | |

### Sistema de control 
| Componente | Observación |
|------------|-------------|
|EPS 32 | Microcontrolador encargado de procesar las señales de entrada y controlar el movimiento de los servomotores. |

### Actuadores 
| Componente | Observación |
|------------|-------------|
|5 Servomotores SG90  |Utilizados para el movimiento individual de cada dedo de la mano. |
| 1 Servomotor MG995  | Encargado de la rotación de la muñeca, proporcionando un mayor torque en comparación con los servomotores de los dedos.   |

### Sensores 
| Componente | Observación |
|------------|-------------|
|Acelerómetro ADXL335  | Sensor utilizado para detectar la rotación en el plano XY, permitiendo generar señales de control para la mano robótica. |

### Sistema electrónico y conexión 
| Componente | Observación |
|------------|-------------|
|PCB (Placa de circuito impreso)  |Permite organizar y conectar de manera estable los componentes electrónicos del sistema. |
| Cables y cables jumper  | Utilizados para realizar las conexiones entre los diferentes componentes del sistema.  |

### Sistema de alimentación
| Componente | Observación |
|------------|-------------|
|Fuente de alimentación (5V – 10A)   |Proporciona la energía necesaria para el funcionamiento del sistema, especialmente para los servomotores que demandan alta corriente.  |
| Cable de alimentación | Permite la conexión de la fuente de energía al sistema. |
|Capacitador cerámico | Utilizado para filtrar el ruido en la alimentación del ESP32, mejorando la estabilidad del sistema.  |
