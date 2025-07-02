# Práctica 7: Protocolo de Enrutamiento Basado en Posición para VANETs

Este proyecto implementa un protocolo de enrutamiento basado en posición para Redes Vehiculares Ad Hoc (VANETs) como parte de la Práctica 7.  
Se extiende el framework de simulación Veins 5.3.1, integrando SUMO para la movilidad y OMNeT++ para la comunicación, con el fin de simular la comunicación vehículo a vehículo (V2V) utilizando una tabla de vecinos mejorada y una selección ponderada del siguiente salto.

---

## 📖 Descripción del Proyecto

El objetivo de este proyecto es diseñar y simular un protocolo de enrutamiento para VANETs que aproveche los datos de posición y factores contextuales para optimizar el reenvío de mensajes. Las mejoras clave incluyen:

- **Tabla de Vecinos Mejorada**  
  Campos adicionales:
  - Distancia al destino  
  - Dirección del vehículo  
  - Número de carril  
  - Conteo de vecinos  

- **Enrutamiento Basado en Posición**  
  Utiliza las posiciones de los vehículos para determinar rutas óptimas.

- **Selección Ponderada del Siguiente Salto**  
  Emplea un mecanismo de puntuación para elegir el camino más adecuado según factores contextuales.

- **Entrega Directa**  
  Prioriza la transmisión directa si el destino es un vecino, evitando cálculos de pesos innecesarios.

- **Entorno de Simulación**  
  - **Veins 5.3.1** para la integración OMNeT++ ↔ SUMO  
  - **SUMO** para simulación de tráfico  
  - **OMNeT++** para simulación de redes

---

## 📂 Estructura del Directorio

```text
p7_rvehiculares/
├── src/
│   └── veins/
│       └── modules/
│           └── application/
│               └── traci/
│                   ├── TraCIDemo11p.cc   # Lógica principal del protocolo
│                   └── beacon.cc          # Soporte para tabla de vecinos extendida
└── examples/
    └── P7/
        ├── omnetpp.ini                  # Configuración de OMNeT++
        ├── erlangen.sumo.cfg            # Configuración de SUMO
        ├── erlangen.net.xml             # Topología de red del escenario
        └── trips.trips.xml              # Rutas de movilidad
