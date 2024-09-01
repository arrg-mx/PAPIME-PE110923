
# Integración de Vuforia en Unity
Para el desarrollo de la aplicación con AR, se integró el SDK de [Vuforia](https://developer.vuforia.com/downloads/sdk) en Unity para permitir el reconocimiento de dos marcadores:

- Marcador de Origen: Representa la posición actual de Raspuma en el mundo real.
- Marcador de Destino: Indica la ubicación objetivo a la cual Raspuma debe dirigirse.

Ambos marcadores fueron añadidos como respectivos GameObject en la escena de Unity para delimitar la interacción. Las imagenes utilizadas como marcadores fueron Codigos QR generados y valorados en el portal de Vuforia para asegurar su reconocimiento.

![UnityProyecto](/RobóticaMóvil/Programas/Raspuma/RaspumaUnityAR-ROS/Images/Marcadores.png)
![UnityProyecto](/RobóticaMóvil/Programas/Raspuma/RaspumaUnityAR-ROS/Images/MarcadoresVuforia.png)

# Script de Unity para la Gestión de Marcadores 

- Funcionalidad General
[MarkerCode](/RobóticaMóvil/Programas/Raspuma/RaspumaUnityAR-ROS/src/Scripts/MarkerManager.cs) Es el script que gestiona la interacción entre la aplicación de realidad aumentada en Unity y Raspuma a través de ROS. Utiliza Vuforia para detectar los marcadores designados.

## Funcionamiento 

- Se declara un LineRenderer para visualizar una línea entre los marcadores en la escena de AR. Esta linea representa la ruta que debe seguir Raspuma para llegar al marcador objetivo. Este elemento es desplegado en AR de tal forma que el entorno visible de Raspuma es detectable por la camara por lo que puede identificar obstaculos en su camino. Debido a la configuración detecta la orientación y posición del marcador, por lo que la ruta es adaptable en tiempo real.

```C#
private void UpdateLine()
{
  lineRenderer.SetPosition(0, markerOrigin.transform.position);
  lineRenderer.SetPosition(1, markerDestination.transform.position);
}
```

- Se inicializan objetos de ROS, incluyendo mensajes TwistMsg que se usarán para enviar comandos a Raspuma.

```C#
//ROS
ros = ROSConnection.GetOrCreateInstance();
ros.RegisterPublisher<RosMessageTypes.Std.StringMsg>("/topic2");
ros.RegisterPublisher<RosMessageTypes.Geometry.TwistMsg>("/robot_twist");
ros.RegisterPublisher<RosMessageTypes.Geometry.TwistMsg>("/robot_camera_twist");
```

- Se suscriben eventos que se activan cuando el estado del marcador cambia (cuando un marcador es detectado o pierde el rastreo) ambos marcadores deben ser detectados para iniciar la ruta de Raspuma y enviar los mensajes correspoindientes al nodo

```C#
originObserver = markerOrigin.GetComponent<ObserverBehaviour>();
destinationObserver = markerDestination.GetComponent<ObserverBehaviour>();
originObserver.OnTargetStatusChanged += OnObserverStatusChanged;
destinationObserver.OnTargetStatusChanged += OnObserverStatusChanged;
```

- Si ambos marcadores están rastreados, se calcula el ángulo entre la dirección de Raspuma (marcador de origen) y la ubicación del marcador de destino

```C#
float desiredTwist = Vector3.SignedAngle(markerOrigin.transform.forward, markerDestination.transform.position- markerOrigin.transform.position, new Vector3(0,1,0));
twist.linear.x = 0.5;
```

- Se crea un mensaje Twist que define la velocidad lineal y angular de Raspuma, y se publica a ROS para que el robot físico lo ejecute.

```C#
twist.angular.z = desiredTwist; 
ros.Publish("/robot_twist", twist);
```

- Si los marcadores no están rastreados, se detiene el movimiento de Raspuma enviando un mensaje Twist vacío.
Métodos de Eventos (OnObserverStatusChanged):

```C#
lineRenderer.enabled = false;
//Send empty Twist msg
twist.linear.x = 0;
twist.angular.z = 0;
ros.Publish("/robot_twist", twist);
```
