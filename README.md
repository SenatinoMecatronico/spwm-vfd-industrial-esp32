# Mini-Variador de Frecuencia (VFD) e Inversor Monofásico SPWM — ESP32 + PLC Siemens LOGO! + SCADA Wi-Fi

Diseño, cálculo, prototipado en placas perforadas FR4 (asistido por **KiCad**) e implementación de firmware en tiempo real para un **Variador de Frecuencia (VFD) / Inversor Monofásico SPWM** con ley de control escalar **Voltaje/Frecuencia ($V/f$) constante** de $0.0\text{ Hz}$ a $60.0\text{ Hz}$.

El sistema integra aislamiento galvánico doble (óptico de alta velocidad para potencia y estándar industrial IEC 61131-2 a $24\text{ VDC}$ para mando desde un **PLC Siemens LOGO! 230RCEo**), interfaz local con pantalla **OLED I2C** y encoder incremental **`KY-040`**, y un servidor **SCADA Web Wi-Fi** embebido para control y telemetría móvil.

---
## 1. Arquitectura Modular del Sistema

El sistema se divide físicamente en dos tarjetas independientes sobre fibra de vidrio (FR4) para garantizar el aislamiento galvánico entre la lógica de control ($3.3\text{ VDC}$), las señales de mando industrial del PLC ($24\text{ VDC}$) y la etapa de conmutación de potencia ($24\text{ VDC} / 220\text{ VAC}$).

### 1.1 Diagrama de Bloques Funcional

```mermaid
flowchart TD
    subgraph CTRL ["1. Tarjeta de Control, HMI y Aislamiento PLC (Dominio 3.3 VDC - GND_ESP)"]
        PLC["PLC Siemens LOGO! 230RCEo<br/>Salidas de Relé Q1, Q2, Q3 (24 VDC)"] -->|Señal 24 VDC| PC817["Aislamiento Óptico de Entrada<br/>3x Optoacopladores PC817 (U1-U3)"]
        PC817 -->|GPIO 32, 33, 18| ESP32["Microcontrolador ESP32 (Dual-Core)<br/>Síntesis DDS 10 kHz + Periférico MCPWM"]
        ENC["Encoder Incremental KY-040<br/>Consigna 0.0 a 60.0 Hz + Marcha/Paro"] -->|GPIO 4, 19, 15| ESP32
        SCADA["Panel SCADA Web Móvil<br/>Wi-Fi SoftAP (192.168.4.1)"] <-->|HTTP / JSON| ESP32
        ESP32 -->|Bus I2C 400 kHz<br/>GPIO 21, 22| OLED["Pantalla OLED 0.96 I2C<br/>Telemetría V/f y Estado PLC"]
    end

    subgraph POW ["2. Tarjeta de Potencia e Inversor SPWM (Dominio Aislado - COM_POT)"]
        ESP32 -->|SPWM 10 kHz + Dead-Time 1.5 us<br/>GPIO 25, 26| OPT6N["Aislamiento de Alta Velocidad<br/>2x Optoacopladores 6N137 (50 ns)"]
        OPT6N -->|HIN, LIN (5 VDC)| IR2110["Driver de Compuerta IR2110<br/>Red Bootstrap (+12 VDC / Diodo 1N4148)"]
        BUS24["Fuente de Potencia Bus DC<br/>24 VDC / 6 A + Fusible 3 A"] --> MOSFET["Etapa Inversora MOSFET<br/>2x IRLZ44N (Configuración Medio Puente)"]
        IR2110 -->|Disparo HO y LO| MOSFET
        MOSFET -->|Onda Conmutada SPWM| FILTRO["Filtro Pasa-Bajos Reactivo LC<br/>Toroides L = 400 uH + C = 1 uF (400 V)"]
        FILTRO -->|Onda Senoidal Limpia<br/>0.0 a 7.3 VAC| TRAFO["Transformador Elevador de Potencia<br/>Salida Variable 0.0 a 167.7 / 220 VAC"]
    end
```

### 1.2 Matriz de Subsistemas e Interfaces de Hardware

| Subsistema / Bloque | Componentes Principales | Interfaz de Entrada | Interfaz de Salida | Dominio Eléctrico |
| :--- | :--- | :--- | :--- | :--- |
| **Aislamiento de Mando Industrial** | 3x Optoacopladores `PC817` (`U1`–`U3`) + $R = 4.7\text{ k}\Omega$ | Contactos `Q1`–`Q3` del PLC Siemens LOGO! ($24\text{ VDC}$) | Entradas digitales `GPIO 32`, `33`, `18` (`INPUT_PULLUP`) | Barrera galvánica $24\text{ VDC} \rightarrow 3.3\text{ VDC}$ |
| **Unidad de Procesamiento y Control** | **ESP32 (38 pines)** (`MCPWM` + `esp_timer` + `WebServer`) | Encoder `KY-040`, entradas PLC y clientes HTTP Wi-Fi | Trenes complementarios SPWM ($10\text{ kHz}$) y bus I2C | Aislado ($3.3\text{ VDC}$ / `GND_ESP`) |
| **Aislamiento de Alta Velocidad** | 2x Optoacopladores lógicos `6N137` + $R_{\text{LED}} = 270\text{ }\Omega$ | Señales `SPWM_H` (`GPIO 25`) y `SPWM_L` (`GPIO 26`) | Nodos `HIN` y `LIN` ($5.0\text{ VDC}$ con *pull-up* de $1\text{ k}\Omega$) | Barrera galvánica $3.3\text{ VDC} \rightarrow 5.0\text{ VDC}$ |
| **Etapa Driver y Elevación Bootstrap** | 1x Driver `IR2110` + Diodo `1N4148` + $C_{\text{boot}} = 10\text{ }\mu\text{F} \parallel 100\text{ nF}$ | Señales lógicas `HIN` (Pin 10) y `LIN` (Pin 12) | Pulsos de compuerta `HO` y `LO` ($12\text{ V}$ / $1.2\text{ A}$ pico) | Referenciado a `COM_POT` y nodo flotante `Vs` |
| **Inversor y Filtro Armónico LC** | 2x MOSFETs `IRLZ44N` + $L = 400\text{ }\mu\text{H}$ + $C = 1\text{ }\mu\text{F}$ ($400\text{ V}$) | Bus continuo de $24\text{ VDC}$ y disparo `HO`/`LO` | Tensión alterna senoidal regulada por curva $V/f$ ($0\text{ a }60\text{ Hz}$) | Potencia (`COM_POT` $\rightarrow$ Salida AC) |
## 1. Arquitectura Modular del Sistema

Para garantizar inmunidad al ruido electromagnético de conmutación ($10\text{ kHz}$) y aislar galvánicamente la lógica de control ($3.3\text{ VDC}$) de la etapa de potencia ($24\text{ VDC} \rightarrow 220\text{ VAC}$), el hardware se divide en dos tarjetas independientes sobre fibra de vidrio FR4:

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

---

## 2. Especificaciones Técnicas de Diseño

| Parámetro del Sistema | Valor Implementado | Detalle |
| :--- | :--- | :--- |
| **Microcontrolador y Síntesis** | **ESP32 (38 pines)** — Periférico `MCPWM` | Síntesis Digital Directa (**DDS**) de 32 bits por interrupción en `IRAM` cada $100\text{ }\mu\text{s}$ (tabla LUT senoidal de 256 muestras). |
| **Frecuencia Portadora ($f_c$)** | **$10.0\text{ kHz}$** ($T_c = 100\text{ }\mu\text{s}$) | Conmutación ultrasónica con **Tiempo Muerto (*Dead-Time*) por hardware de $1.5\text{ }\mu\text{s}$** en ambos flancos. |
| **Rango de Frecuencia ($f_m$)** | **$0.0\text{ Hz}$ a $60.0\text{ Hz}$** (Resolución: $0.5\text{ Hz}$) | Corte automático de pulsos por debajo de $1.0\text{ Hz}$ para evitar inyección DC en el transformador. |
| **Rampa Industrial** | **$15.0\text{ Hz/s}$** (Aceleración y Frenado) | Arranque y desaceleración progresiva en rampa antes de deshabilitar la etapa de potencia. |
| **Aislamiento de Potencia** | **Optoacopladores rápidos `6N137`** | Retardo de propagación de $50\text{ ns}$ configurados en modo sumidero (*Sink*) no inversor. |
| **Etapa Driver y Conmutación** | **`IR2110` + MOSFETs `IRLZ44N`** | Red *Bootstrap* con diodo rápido `1N4148` / `UF4007`, ciclo acotado ($4\%$ a $96\%$) y rutina de precarga de $5\text{ ms}$. |
| **Filtro de Salida Primario** | **Filtro Reactivo LC ($L = 400\text{ }\mu\text{H}$, $C = 1\text{ }\mu\text{F}$)** | 2 inductores toroidales de $200\text{ }\mu\text{H}$ en serie + capacitor de polipropileno de $1\text{ }\mu\text{F}$ ($400\text{ V}$). |
| **Integración con PLC** | **Siemens LOGO! 230RCEo** ($24\text{ VDC}$) | 3 entradas aisladas con `PC817`: Marcha/Paro (`Q1`) y 3 velocidades preseleccionadas (`20 Hz`, `40 Hz`, `60 Hz` vía `Q2` y `Q3`). |

---

## 3. Cálculos Principales

### 3.1 Polarización de los Optoacopladores Rápidos 6N137 ($R_{\text{LED}} = 270\text{ }\Omega$)
Para garantizar una conmutación limpia a $10\text{ kHz}$ sin superar el límite de corriente de los pines GPIO del ESP32 ($3.3\text{ V}$) y considerando $V_F \approx 1.4\text{ V}$:

$$I_F = \frac{V_{\text{GPIO}} - V_F}{R_{\text{LED}}} = \frac{3.3\text{ V} - 1.4\text{ V}}{270\text{ }\Omega} \approx 7.04\text{ mA}$$

### 3.2 Corriente Pico de Disparo de Compuerta en el IR2110 ($R_G = 10\text{ }\Omega$)
Alimentando el riel $V_{CC}$ del driver `IR2110` con $12.0\text{ VDC}$:

$$I_{G(\text{pico})} = \frac{V_{CC}}{R_G} = \frac{12.0\text{ V}}{10\text{ }\Omega} = 1.2\text{ A}$$

### 3.3 Aislamiento de Entradas de $24\text{ VDC}$ desde el PLC LOGO! ($R_{\text{PLC}} = 4.7\text{ k}\Omega$)

$$I_{F(\text{PC817})} = \frac{24.0\text{ V} - 1.2\text{ V}}{4700\text{ }\Omega} \approx 4.85\text{ mA} \quad \Big| \quad P_R = (0.00485\text{ A})^2 \times 4700\text{ }\Omega \approx 0.11\text{ W}$$

### 3.4 Diseño del Filtro Reactivo LC ($L = 400\text{ }\mu\text{H}$, $C = 1.0\text{ }\mu\text{F}$)
* **Reactancia a la frecuencia fundamental ($60\text{ Hz}$):**
  $$X_{L(60\text{ Hz})} = 2\pi \cdot (60\text{ Hz}) \cdot (400 \times 10^{-6}\text{ H}) = 0.151\text{ }\Omega$$
* **Reactancia a la frecuencia portadora ($10\text{ kHz}$):**
  $$X_{L(10\text{ kHz})} = 2\pi \cdot (10000\text{ Hz}) \cdot (400 \times 10^{-6}\text{ H}) = 25.13\text{ }\Omega$$
* **Frecuencia de corte del filtro LC primario:**
  $$f_c = \frac{1}{2\pi\sqrt{L \cdot C}} = \frac{1}{2\pi\sqrt{400 \times 10^{-6}\text{ H} \cdot 1 \times 10^{-6}\text{ F}}} \approx 7.95\text{ kHz}$$

---

## 4. Evolución Experimental y Mediciones en Banco (multímetro UNI-T UT89XD)

Durante el desarrollo se registraron cuatro iteraciones de hardware, documentando el impacto real del medio de montaje y del filtrado de salida a $60.0\text{ Hz}$:

| Iteración de Hardware | Configuración de Salida y Filtro | Voltaje AC Baja (Salida Inversor) | Voltaje AC Alta (Secundario Trafo) | Observación Técnica en Laboratorio |
| :--- | :--- | :---: | :---: | :--- |
| **Iteración 1: Protoboard** | Sin filtro LC, conexiones con jumpers | $\sim 1.0\text{ V} \rightarrow 0\text{ V}$ | $0.0\text{ VAC}$ | Bloqueo UVLO del Bootstrap por resistencia de contacto e inductancia parásita en protoboard. |
| **Iteración 2: Placa FR4 Potencia (v1)** | Acople $1000\text{ }\mu\text{F}$ + Resistencia serie $100\text{ }\Omega$ ($5\text{ W}$) en toma $12\text{ V}$ | **$7.83\text{ VAC}$** *(Teórico: $7.80\text{ V}$, Error: $0.38\%$)* | **$59.87\text{ VAC}$** | La resistencia de $100\text{ }\Omega$ actuaba como divisor de tensión quedándose con $\sim 3.7\text{ V}$. Lectura de $9.3\text{ kHz}$ por rizado. |
| **Iteración 3: Placa FR4 Potencia (v2)** | $2\times 100\text{ }\Omega$ en paralelo ($50\text{ }\Omega$) + Capacitor `CL21 0.047uF` en toma $9\text{ V}$ | $7.83\text{ VAC}$ | **$109.8\text{ VAC}$** | El capacitor `CL21` absorbió el rizado de la portadora en alta tensión estabilizando la lectura en **$60.0\text{ Hz}$**. |
| **Iteración 4: Tarjeta 1 + Placa Potencia + Filtro LC** | **2 Toroides de $200\text{ }\mu\text{H}$ en serie ($400\text{ }\mu\text{H}$) + $1\text{ }\mu\text{F}$ ($400\text{ V}$)** | **$7.30\text{ VAC}$** (en devanado) | **$167.7\text{ VAC}$** | Eliminación de pérdidas resistivas por calor (+57.9 VAC adicionales), control PLC LOGO! y SCADA Wi-Fi activos. |

---

## 5. Bitácora de Fallas y Soluciones (Troubleshooting)

1. **Tensión fija de $1.56\text{ V}$ y $0.77\text{ V}$ en el Pin 6 (`Vo`) de los optoacopladores `6N137`:**
   * *Diagnóstico:* Al realizar pruebas de continuidad en frío sobre la primera placa soldada, se detectó una unión abierta entre la resistencia *pull-up* `R3` ($1\text{ k}\Omega$) y el Pin 6 de `U1`. Al ser una salida de colector abierto, el nodo `HIN` quedaba flotante e incapaz de excitar al `IR2110`.
   * *Solución:* Resoldado del nodo y establecimiento de un protocolo obligatorio de verificación de continuidad punto a punto antes de insertar los CIs en sus zócalos.
2. **Disparo de la protección OCP (apagado de la fuente de $24\text{ VDC}$) al pulsar `STOP` o por vibración:**
   * *Diagnóstico:* Dos causas combinadas: (a) En software, la función de paro deshabilitaba el generador de *Dead-Time* (`mcpwm_deadtime_disable`) antes de que finalizara el ciclo de la interrupción de $10\text{ kHz}$; (b) En hardware, cualquier micro-desconexión del jumper de `3V3` en el protoboard apagaba ambos LEDs de los `6N137`, enviando $+5\text{ V}$ simultáneos a `HIN` y `LIN` y provocando conducción cruzada (*Shoot-Through*).
   * *Solución:* Migración del ESP32 a la **Tarjeta 1 soldada con borneras `KF301`** y adición de una espera de seguridad de $200\text{ }\mu\text{s}$ (2 ciclos de portadora) forzando ambos canales a `LOW` antes de retirar el *Dead-Time*.
3. **Lectura errónea de $9.300\text{ kHz}$ en el multímetro en lugar de $60.0\text{ Hz}$:**
   * *Diagnóstico:* Con el secundario del transformador en vacío, el rizado residual de la portadora de $10\text{ kHz}$ provocaba múltiples cruces por cero alrededor de $0\text{ V}$, engañando al frecuencímetro del multímetro digital.
   * *Solución:* Implementación del filtro pasa-bajos LC ($400\text{ }\mu\text{H} + 1\text{ }\mu\text{F}$ en baja tensión y $0.047\text{ }\mu\text{F}$ a $630\text{ V}$ en alta tensión).
4. **Apagado abrupto de la lámpara piloto durante la rampa de desaceleración:**
   * *Diagnóstico:* La condición de habilitación exigía `currentFreqHz >= 1.0 && runRequested`, cortando los MOSFETs en el instante en que `runRequested` pasaba a `false` aunque la frecuencia siguiera descendiendo en rampa.
   * *Solución:* Se vinculó el corte de potencia exclusivamente al umbral de frecuencia (`currentFreqHz >= 1.0 Hz`), logrando una atenuación progresiva hasta $1.0\text{ Hz}$.

---

## 6. Próxima Etapa (En Desarrollo)
* **Arquitectura Industrial Directa ($310\text{ VDC} \rightarrow 220\text{ VAC}$ sin transformador de $60\text{ Hz}$):** Etapa rectificadora con puente `KBPC3510`, circuito de precarga de Bus DC con termistor NTC y relé de bypass, Puente H completo de alta tensión con 4 MOSFETs **`IRF840` ($500\text{ V}$)** y diodos **`UF4007`**, y lazo cerrado de tensión/corriente mediante sensores **`ZMPT101B`** y **`ZMCT103C`**.
