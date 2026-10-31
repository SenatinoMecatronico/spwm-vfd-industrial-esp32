# ⚡ Mini-Variador de Frecuencia (VFD) e Inversor Monofásico SPWM — ESP32 + PLC Siemens LOGO! + SCADA Wi-Fi

Diseño, cálculo, prototipado en placas perforadas FR4 (asistido por **KiCad** + **Specctra DSN**) e implementación de firmware en tiempo real para un **Variador de Frecuencia (VFD) / Inversor Monofásico SPWM** con ley de control escalar **Voltaje/Frecuencia ($V/f$) constante** de $0.0\text{ Hz}$ a $60.0\text{ Hz}$.

El sistema integra aislamiento galvánico doble (óptico de alta velocidad para potencia y estándar industrial IEC 61131-2 a $24\text{ VDC}$ para mando desde un **PLC Siemens LOGO! 230RCEo**), interfaz local con pantalla **OLED I2C** y encoder incremental **`KY-040`**, y un servidor **SCADA Web Wi-Fi** embebido para control y telemetría móvil.

---
  
## 🏗️ 1. Arquitectura Modular del Sistema

Para garantizar inmunidad al ruido electromagnético de conmutación ($10\text{ kHz}$) y aislar galvánicamente la lógica de control ($3.3\text{ VDC}$) de la etapa de potencia ($24\text{ VDC} \rightarrow 220\text{ VAC}$), el hardware se divide en dos tarjetas independientes sobre fibra de vidrio FR4:

```text
+-----------------------------------------------------------------------------------+
| TARJETA 1: UNIDAD DE CONTROL, HMI Y AISLAMIENTO PLC (Matriz 30x42 - 3.3V Aislados)|
|                                                                                   |
|  [PLC LOGO! 230RCEo] --(24VDC Q1..Q3)--> [3x PC817 (U1..U3)] ---> GPIO 32, 33, 18 |
|  [Encoder KY-040]    --(Cuadratura)-----------------------------> GPIO 4, 19, 15  |
|  [Celular / PC]      <--(Wi-Fi SoftAP: 192.168.4.1)-------------> [ESP32 DualCore]|
|  [Pantalla OLED]     <--(I2C 400 kHz)---------------------------- GPIO 21, 22     |
+-------------------------------------------------------------------------+---------+
                                                                          |
                                       Señales SPWM (10 kHz + Dead-Time 1.5 us)
                                                                          |
+-------------------------------------------------------------------------v---------+
| TARJETA DE POTENCIA: AISLAMIENTO RÁPIDO, DRIVER BOOTSTRAP E INVERSOR (COM_POT)    |
|                                                                                   |
|  GPIO 25/26 ---> [2x Optoacopladores 6N137 (50ns)] ---> [Driver IR2110 + Bootstrap]|
|                                                                   |               |
|  [Bus 24 VDC] ---> [Puente MOSFET IRLZ44N (Q1, Q2)] <-------------+               |
|                            |                                                      |
|                            v                                                      |
|             [Filtro Reactivo LC: 400 uH + 1 uF 400V] ---> [Trafo Elevador 220VAC] |
+-----------------------------------------------------------------------------------+
