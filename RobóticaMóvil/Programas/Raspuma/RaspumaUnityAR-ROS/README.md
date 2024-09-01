# RaspumaUnityAR-ROS

<p align="center"><img src="images/warehouse.gif"/></p>

<!-- [![Version](https://img.shields.io/github/v/tag/Unity-Technologies/Unity-Robotics-Hub)](https://github.com/Unity-Technologies/Unity-Robotics-Hub/releases) -->
![ROS](https://img.shields.io/badge/ROS-Humble-brightgreen)
![Unity](https://img.shields.io/badge/Unity-2022.3+-brightgreen)
![Vuforia](https://img.shields.io/badge/Vuforia-10.25+-brightgreen)

# Objetivo

El objetivo de este proyecto es permitir que los usuarios puedan visualizar en Realidad Aumentada (AR) la ruta de movimiento del robot móvil "Raspuma" hacia un marcador objetivo. A través de la integración de Unity, ROS2, y Vuforia, se logra una interacción fluida entre el mundo físico, y la virtualización de rutas, facilitando el control e identificación de direcciones en tiempo real del robot.

# Descripción

Raspuma es un robot móvil diseñado para experimentar y desarrollar algoritmos de navegación autónoma. Equipado con sensores ultrasónicos, Raspuma es capaz de detectar y evitar obstáculos en su entorno. Este proyecto combina las capacidades de ROS (Robot Operating System) para el control del robot, Unity para la creación de aplicaciones de realidad aumentada (AR) y ambientes virtuales, y Vuforia para el reconocimiento de marcadores físicos.

Se desarrolló una aplicación de Unity para Android que utiliza Vuforia para identificar dos marcadores específicos:

- Marcador de Origen: Representa la posición actual de Raspuma en el mundo real.
- Marcador de Destino: Indica la ubicación objetivo hacia la cual Raspuma debe moverse.

Cuando la aplicación detecta ambos marcadores, dibuja una línea en AR que marca la ruta que Raspuma debe seguir. La comunicación entre Unity y ROS permite que Raspuma se mueva en la dirección identificada por el marcador de destino, incluso si el marcador objetivo cambia de posición u orientación en el entorno en tiempo real. De esta manera, los usuarios pueden  visualizar el comportamiento de navegación de Raspuma de acuerdo a los componentes fisicos.

# Contenido
- [Objetivo](#objetivo)
- [Descripción](#descripción)
- [Desarrollo](#desarrollo)
- [Ejecución](#ejecución)
- [Referencias](#referencias)

## Desarrollo

El proyecto utiliza tres componentes principales; la integración de la comunicación de Unity con ROS de basado en el repositorio de [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) de Unity Technologies, la creación del nodo EndPoint con [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2) y la integración del SDK de [Vuforia](https://developer.vuforia.com/downloads/sdk) para la virtualización de la ruta en Realidad Aumentada.


| Enlace | Descripción |
|---|---|
| [ROS-TCP-Connector](ROS–Unity.md) | Establecer el entorno en ROS2 para la comunicación con Unity|
| [ROS-TCP-Endpoint](ROS_EndPoint.md) | Configuración del nodo EndPoint para recepción de mensajes|
| [Vuforia](Unity-Vuforia.md) | Configuración de SDK de Vuroia y control de marcadores|

## Ejecución

### Ejecutar ROS:

- Lanzar el nodo del ROS TCP Endpoint:

``` ROS
ros2 run ros_tcp_endpoint default_server_endpoint
```
- Ejecuta el nodo del hardware de Raspuma:

``` ROS
ros2 run hardware_communication hardware_node
```

### Ejecutar Unity:

- Abrir el proyecto en [Unity](https://github.com/ZaaRamirez/RaspumaUnity)
- Asegurarse de que la configuración del ROSConnection esté apuntando a la IP y puerto correctos.
- Ejecutar la escena principal que contiene los componentes de Vuforia y ROS.

### Ejecutar la aplicación Andorid

- Construir y despliegar la aplicación en un dispositivo Android.
- Utilizar la cámara del dispositivo para reconocer los marcadores y comenzar a controlar a Raspuma en tiempo real.

## Referencias

- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) de Unity Technologies: Base del proyecto para la comunicación entre Unity y ROS.
- [Documentación de Vuforia](https://developer.vuforia.com/library/getting-started/vuforia-engine-package-unity): Para la implementación de reconocimiento de imágenes en Unity.
- [ROS Humble](https://docs.ros.org/en/humble/Installation.html): Sistema operativo para robots utilizado en el control de Raspuma.
- [GPIO Zero Documentation](https://gpiozero.readthedocs.io/en/stable/): Para el control de hardware en Raspberry Pi.
- [Unity Documentation](https://docs.unity3d.com/es/2018.4/Manual/vuforia-sdk-overview.html): Recursos y guías para el desarrollo en Unity.

