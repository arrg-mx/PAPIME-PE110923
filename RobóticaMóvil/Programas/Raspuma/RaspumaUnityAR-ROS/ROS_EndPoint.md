# Instalación del ROS-TCP-Endpoint:

ROS TCP Endpoint es el paquete utilizado para crear un nodo para aceptar mensajes enviados desde una escena de Unity que tenga configurado ROS TCP Connector. Para la integración de esta paquete se siguieron las instrucciones correspodientes al entorno de ROS2 descritas en la documentación [ROS TCP Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2).

- Para poder utilizar este paquete, es necesario primero crear el entorno de ROS2 en Raspuma
```
raspuma_ws
```
- Dentro del entorno, se descarga la rama ROS2 del repositorio [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2) y se copia en la carpeta src del espacio de trabajo. Dentro se compila ejecutendo los siguientes comandos:

```bash
source install/setup.bash
colcon build
source install/setup.bash
```
- Para establecer la comunicación, estando dentro del entonrno se deben correr el siguiente comando en una terminal que en constante ejecución estara recibiendo los mensajes:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint
```

# Ejecución del Nodo del Robot:

[hardware_node.py](/src/hardware_communication) Este script se ejecuta en Raspuma y controla los motores, servos y sensores ultrasónicos del robot. Recibe comandos de movimiento desde Unity a través de ROS y ajusta la locomoción y orientación de Raspuma en consecuencia. Además, mide distancias a obstáculos y publica estos datos para su uso futuro.

```bash
ros2 run hardware_communication hardware_node
```
- Recibir Comandos de Movimiento: A través de los tópicos ```/robot_twist/ robot_camera_twist ```, el nodo recibe comandos de velocidad y orientación desde Unity.
- Controlar la Locomoción: Ajusta la velocidad y dirección de los motores para mover Raspuma hacia el destino.
- Ajustar la Orientación de la Cámara: Controla los servos que manipulan la cámara en base a los comandos recibidos.
- Publicar Distancias: Envía información de los sensores ultrasónicos a través del tópico ```/robot_us ```  para la detección de obstáculos.
