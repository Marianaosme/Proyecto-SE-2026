# Requisitos y Plan de testing 

Este documento presenta los requisitos tanto funcionales como no funcionales del proyecto, así como sus respectivo plan de pruebas (testing). Con objetivo es definir el comportamiento esperado del sistema y establecer criterios que permitan validar su correcto funcionamiento tanto a nivel mecánico como electrónico y de software.

## Requisitos Funcionales

| ID | Requisito Funcional | Plan de Testing | Resultado Esperado |
|----|----------------------|-----------------|-----------------|
| RF-01 | Los servos de los dedos se mueven en un rango definido y sin obstrucciones mecánicas. | Ejecutar un programa de control que varíe la señal del servo usando un potenciómetro. Evaluar empíricamente el rango mínimo y máximo de movimiento. | Los servos deben moverse suavemente dentro del rango establecido sin presentar bloqueos, ruidos anormales o limitaciones físicas. |
| RF-02 | Los potenciómetros de los dedos se mueven en el mismo rango que los servos de los dedos. | Medir con un multímetro la variación de voltaje del potenciómetro mientras se mueve en todo su recorrido. | El rango de voltaje del potenciómetro debe corresponder proporcionalmente al rango de movimiento del servo.|
| RF-03 | El servo de la muñeca se mueve en un rango definido y sin obstrucciones mecánicas. | Ejecutar un programa de control utilizando un potenciómetro para variar la posición del servo. Determinar empíricamente los límites de movimiento y verificar posibles obstrucciones. | El servo de la muñeca debe moverse de forma continua y controlada dentro de su rango sin interferencias mecánicas.|
| RF-04 | El acelerómetro reconoce y registra los movimientos en los ejes X y Y. | Ejecutar el programa en el entorno de desarrollo y visualizar en la terminal los valores. | Los valores mostrados en la terminal deben ser proporcionales al movimiento realizado en cada eje. |
| RF-05 | El diseño mecánico de la mano robótica debe ser análogo a una mano humana. | Realizar pruebas empíricas verificando que la flexo-extensión de los dedos y la rotación de la muñeca sean adecuadas. | La mano robótica debe replicar de forma adecuada los movimientos básicos, con rangos y comportamientos similares. |
| RF-06 | Cada componente físico de la mano robótica debe ensamblarse de forma funcional. | Ensamblar todos los componentes del sistema y verificar que el conjunto permita movimientos continuos y sin interferencias. Evaluar posibles desajustes o fallas mecánicas. | El sistema ensamblado debe presentar un movimiento fluido. |
| RF-07 | El material de la mano robótica debe resistir su propio peso. | Aplicar una carga de compresión equivalente al peso total de la estructura y observar su comportamiento mecánico. | La estructura debe soportar su propio peso sin deformaciones permanentes ni fallas estructurales. |
| RF-08 | La interfaz de usuario permite configurar los movimientos de la mano robótica. | Verificar que los comandos enviados desde la interfaz correspondan correctamente a los movimientos ejecutados por la mano robótica. Realizar pruebas variando parámetros y observando la respuesta del sistema. | Los movimientos ejecutados por la mano robótica deben ser coherentes con la configuración definida en la interfaz de usuario. |
| RF-09 | La interfaz de usuario permite visualizar el grado de los movimientos de la mano robótica. | Comparar los valores mostrados en la interfaz con los rangos reales de movimiento de la mano robótica mediante pruebas controladas. | La información visualizada en la interfaz debe reflejar de manera precisa los ángulos de la mano robótica. |
| RF-10 | La interfaz de usuario permite visualizar errores o eventos relevantes del sistema. | Generar condiciones de error o eventos y verificar que la interfaz los detecte y los muestre correctamente. | La interfaz debe mostrar información clara y coherente sobre errores o eventos relevantes del sistema. |



## Requisitos No Funcionales

| ID | Requisito No Funcional | Plan de Testing |
|----|----------------------|-----------------|
| RNF-01 | La fuente de alimentación del sistema debe soportar la carga demandada por los componentes. | Medir con un multímetro la corriente consumida y verificar que la fuente de alimentación sea capaz de suministrar la corriente requerida. |
| RNF-02 | La corriente entregada al ESP32 no debe sobrepasar el límite de seguridad del sistema. | Medir con un multímetro la corriente de salida hacia el ESP32 durante la operación del sistema. |
| RNF-03 | El sistema de seguridad debe proteger los motores ante sobrecargas. | Aplicar una carga superior al límite de seguridad a un servo y verificar que el sistema detecte la sobrecarga y corte la alimentación del motor. | 
| RNF-04 | El sistema debe identificar los errores que se muestran en la interfaz. | Generar diferentes condiciones de error en el sistema y verificar que estos sean detectados y reflejados correctamente en la interfaz de usuario. |


