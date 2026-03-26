# Requisitos y Plan de testing 

## Requisitos Funcionales

| ID | Requisito Funcional | Plan de Testing |
|----|----------------------|-----------------|
| RF-01 | Los servos de los dedos se mueven en un rango definido y sin obstrucciones mecánicas. | Ejecutar un programa de control que varíe la señal del servo usando un potenciómetro. Evaluar empíricamente el rango mínimo y máximo de movimiento. Verificar visualmente que no existan bloqueos o interferencias mecánicas. |
| RF-02 | Los potenciómetros de los dedos se mueven en el mismo rango que los servos de los dedos. | Medir con un multímetro la variación de voltaje del potenciómetro mientras se mueve en todo su recorrido. |
| RF-03 | El servo de la muñeca se mueve en un rango definido y sin obstrucciones mecánicas. | Ejecutar un programa de control utilizando un potenciómetro para variar la posición del servo. Determinar empíricamente los límites de movimiento y verificar posibles obstrucciones. | 
| RF-04 | El acelerómetro reconoce y registra los movimientos en los ejes X y Y. | Ejecutar el programa en el entorno de desarrollo y visualizar en la terminal los valores. |
| RF-05 | El diseño mecánico de la mano robótica debe ser análogo a una mano humana. | Realizar pruebas empíricas verificando que la flexo-extensión de los dedos y la rotación de la muñeca sean adecuadas. |
| RF-06 | Cada componente físico de la mano robótica debe ensamblarse de forma funcional. | Ensamblar todos los componentes del sistema y verificar que el conjunto permita movimientos continuos y sin interferencias. Evaluar posibles desajustes o fallas mecánicas. |
| RF-07 | El material de la mano robótica debe resistir su propio peso. | Aplicar una carga de compresión equivalente al peso total de la estructura y observar su comportamiento mecánico. | 
| RF-08 | La interfaz de usuario permite configurar los movimientos de la mano robótica. | Verificar que los comandos enviados desde la interfaz correspondan correctamente a los movimientos ejecutados por la mano robótica. Realizar pruebas variando parámetros y observando la respuesta del sistema. |
| RF-09 | La interfaz de usuario permite visualizar el grado de los movimientos de la mano robótica. | Comparar los valores mostrados en la interfaz con los rangos reales de movimiento de la mano robótica mediante pruebas controladas. |
| RF-10 | La interfaz de usuario permite visualizar errores o eventos relevantes del sistema. | Generar condiciones de error o eventos y verificar que la interfaz los detecte y los muestre correctamente. |


## Requisitos No Funcionales

| ID | Requisito No Funcional | Plan de Testing |
|----|----------------------|-----------------|
| RNF-01 | Los servos de los dedos se mueven en un rango definido y sin obstrucciones mecánicas. | Ejecutar un programa de control que varíe la señal del servo usando un potenciómetro. Evaluar empíricamente el rango mínimo y máximo de movimiento. Verificar visualmente que no existan bloqueos o interferencias mecánicas. |
| RNF-02 | Los potenciómetros de los dedos se mueven en el mismo rango que los servos de los dedos. | Medir con un multímetro la variación de voltaje del potenciómetro mientras se mueve en todo su recorrido. |
| RNF-03 | El servo de la muñeca se mueve en un rango definido y sin obstrucciones mecánicas. | Ejecutar un programa de control utilizando un potenciómetro para variar la posición del servo. Determinar empíricamente los límites de movimiento y verificar posibles obstrucciones. | 
| RNF-04 | El acelerómetro reconoce y registra los movimientos en los ejes X y Y. | Ejecutar el programa en el entorno de desarrollo y visualizar en la terminal los valores. |


