# Sistema Multimodal de Monitorización Fisiológica para Optimización de Terapia con Insulina

## Descripción General
El proyecto consiste en el desarrollo de un sistema wearable para monitorización continua de parámetros fisiológicos que afectan al control glucémico en pacientes diabéticos. El sistema integra múltiples sensores en una pulsera inteligente que procesa y fusiona datos en tiempo real para optimizar la dosificación de insulina.

## Innovación y Justificación
Las bombas de insulina actuales basan sus cálculos principalmente en niveles de glucosa y carbohidratos ingeridos. Sin embargo, factores como el estrés, la actividad física y el estado fisiológico general afectan significativamente al control glucémico. Este sistema proporciona una monitorización multimodal que permite medir la actividad física para mejorar el ajuste dinámico de los parámetros de infusión.

## Arquitectura del Sistema

### Hardware
1. **Microcontrolador Principal**: NRF52840
   - Conectividad Bluetooth LE
   - Capacidad de procesamiento para algoritmos de fusión de sensores
   - Gestión de energía eficiente

2. **Sensores Integrados**:
   - MAX30100: PPG para ritmo cardíaco
   - BNO055: IMU para detección de actividad
   - Módulo de temperatura corporal

### Firmware (Foco Principal)
1. **Sistema de Adquisición**:
   - Drivers de bajo nivel para cada sensor
   - Gestión de timing y sincronización
   - DMA para transferencias eficientes
   - Filtrado en tiempo real

2. **Procesamiento de Señales**:
   - Algoritmos de reducción de ruido
   - Detección de movimiento
   - Fusión de datos de múltiples sensores
   - Extracción de características fisiológicas

3. **Sistema de Control**:
   - Máquina de estados finitos para gestión de modos
   - Sistema de eventos para manejo de alarmas
   - Gestión de energía adaptativa
   - Buffer circular para datos históricos

4. **Comunicación**:
   - Stack BLE personalizado
   - Protocolo de comunicación robusto
   - Gestión de conexiones múltiples
   - Compresión de datos

### Aspectos Técnicos Destacados
1. **Desarrollo de Firmware**:
   - Implementación de drivers personalizados
   - Optimización de rutinas críticas en tiempo
   - Gestión de memoria eficiente
   - Debugging de sistemas en tiempo real

2. **Procesamiento Embebido**:
   - Algoritmos de filtrado adaptativo
   - Fusión de sensores
   - Detección de patrones de actividad
   - Análisis de variabilidad cardíaca

3. **Optimización**:
   - Consumo de energía
   - Latencia de procesamiento
   - Uso de memoria
   - Robustez del sistema

## Fases de Desarrollo

1. **Fase 1: Plataforma Base**
   - Configuración del microcontrolador
   - Implementación de drivers básicos
   - Sistema de gestión de energía

2. **Fase 2: Integración de Sensores**
   - PPG/ECG para ritmo cardíaco
   - IMU para actividad

3. **Fase 3: Procesamiento**
   - Algoritmos de filtrado
   - Fusión de sensores
   - Detección de eventos

4. **Fase 4: Comunicación**
   - Implementación BLE
   - Protocolo de datos I2C
     
5. **Fase 5: Representación**
   - Gráfica de representación de datos.

## Extensiones Futuras
- EDA
- Conexión con APIs de salud (HealthKit/Google Fit)
- Algoritmos avanzados de predicción
