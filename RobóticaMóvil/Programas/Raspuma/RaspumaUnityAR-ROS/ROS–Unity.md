# ROS-TCP-Connector

El ROS-TCP-Connector es un paquete de Unity que facilita la conexión y comunicación entre un proyecto de Unity y un entorno ROS. Para este proyecto se siguieron los pasos descritos en [Unity Setup](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md) para añadir dicho paquete a un proyecto nuevo de Unity. De forma general se tienen las siguientes indicaciones:

- Proyecto de Unity:

Crear un nuevo proyecto de Unity 3D, en este caso se tomó la versión 2022.3.7f1, pero se considera compatible para versiones superiores.

![UnityProyecto](/RobóticaMóvil/Programas/Raspuma/RaspumaUnityAR-ROS/Images/UnityProyecto.png)

- Instalación del Paquete:

Se importa el paquete ROS-TCP-Connector en el proyecto Unity desde el gestor de paquetes (Package Manager). añadiendo la URL del repositorio [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector).

![UnityProyecto](/RobóticaMóvil/Programas/Raspuma/RaspumaUnityAR-ROS/Images/URL.png)
![UnityProyecto](/RobóticaMóvil/Programas/Raspuma/RaspumaUnityAR-ROS/Images/URL2.png)

- Configuración del ROS Settings:

Instalado el nuevo paquete, en la barra de herramientas desde la ruta ROS > ROS Settings, se configura la dirección IP correspondiente a la del robot Raspuma y el puerto por defecto que se utiliza para la comunicación con ROS. De la misma forma se seleciona el protocolo de ROS2.

![UnityProyecto](/RobóticaMóvil/Programas/Raspuma/RaspumaUnityAR-ROS/Images/Protocolo.png)

- Creación del ROSConnection:

Dentro de la escena de Unity se crea el objeto de controlador que integra el componente ROSConnection, donde se configura los tópicos y mensajes de ROS que Unity necesita publicar en este componente.

![UnityProyecto](/RobóticaMóvil/Programas/Raspuma/RaspumaUnityAR-ROS/Images/Controller.png)
![UnityProyecto](/RobóticaMóvil/Programas/Raspuma/RaspumaUnityAR-ROS/Images/Controller2.png)
