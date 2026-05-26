# Matriz De Trazabilidad De Requerimientos (SRTM)
La siguiente matriz relaciona cada requisito con su respectivo caso de prueba y tipo de ensayo, permitiendo verificar la cobertura del proceso de validación del sistema.

| Requirement ID | Requirement | Test Case ID | Test Type | Artifacts/Evidence |
|----------------|-------------|---------------|------------|--------------------|
| RF-001 | Los servos de los dedos se mueven en un rango definido y sin obstrucciones mecánicas.  | TC-001 | Unit Test | Registro de rangos de movimiento |
| RF-002 | Los servos se deben mover en un rango proporcional establecido por los potenciómetros. | TC-002 | Integration Test | Resultados de calibración |
| RF-003 | El servo de la muñeca se mueve en un rango definido y sin obstrucciones mecánicas. | TC-003 | Unit Test | Fotografías |
| RF-004 | El giroscopio reconoce y registra los movimientos en los ejes X y Y.  | TC-004 | Unit Test | Capturas del monitor serial |
| RF-005 | El diseño mecánico de la mano robótica debe ser análogo a una mano humana.  | TC-005 | System Test | Fotografías |
| RF-006 | El material de la mano robótica debe resistir su propio peso.  | TC-006 | Integration Test | Fotografías |
| RF-007 | La interfaz de usuario permite configurar los movimientos de la mano robótica.  | TC-007 | Integration Test | Capturas de interfaz, y fotografías del movimiento |
| RF-008 | La interfaz de usuario permite visualizar errores o eventos relevantes del sistema. | TC-008 | System Test | Capturas de errores en interfaz |
| RF-009 | El sistema debe replicar movimientos en configuración espejo.  | TC-009 | HIL / Integration Test | Fotografías |
| RF-010 | El controlador debe comenzar el proceso de calibración de manera exitosa con apoyo de la información presentada en la pantalla de este. | TC-010 | Integration Test | Capturas de calibración  |
| RF-011 | El mensaje presentado en la pantalla del controlador debe ser coherente con el estado actual del controlador.  | TC-011 | Integration Test | Fotografías de pantalla |
| RNF-001 | La fuente de alimentación del sistema debe soportar la carga demandada por los componentes.  | TC-101 | HIL Test | Fotografías y mediciones |
| RNF-002 | La corriente entregada al ESP32 no debe sobrepasar el límite de seguridad del sistema. | TC-102 | Unit Test | Fotografías |
| RNF-003 | El sistema debe mantener una conexión inalámbrica estable entre el controlador y la mano.  | TC-103 | HIL / Integration Test | Monitor serial y fotos |
| RNF-004 | El sistema debe identificar los errores que se muestran en la interfaz. | TC-104 | System Test | Capturas de errores |
