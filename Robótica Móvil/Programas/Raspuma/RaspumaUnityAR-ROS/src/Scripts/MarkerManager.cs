using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Vuforia;
using Unity.Robotics.ROSTCPConnector;

public class MarkerManager : MonoBehaviour
{
    public GameObject markerOrigin;
    public GameObject markerDestination;
    public LineRenderer lineRenderer;

    private ObserverBehaviour originObserver;
    private ObserverBehaviour destinationObserver;
    //ROS
    public ROSConnection ros;
    RosMessageTypes.Geometry.TwistMsg twist = new();
    RosMessageTypes.Geometry.TwistMsg twistCam = new();
    void Start()
    {
        originObserver = markerOrigin.GetComponent<ObserverBehaviour>();
        destinationObserver = markerDestination.GetComponent<ObserverBehaviour>();

        originObserver.OnTargetStatusChanged += OnObserverStatusChanged;
        destinationObserver.OnTargetStatusChanged += OnObserverStatusChanged;

        lineRenderer.positionCount = 2;
        lineRenderer.enabled = false;

        //ROS
        ros = ROSConnection.GetOrCreateInstance();
        //ROSConnection.GetOrCreateInstance().Subscribe<RosMessageTypes.Std.StringMsg>("/topic2", ColorChange);
        ros.RegisterPublisher<RosMessageTypes.Std.StringMsg>("/topic2");
        ros.RegisterPublisher<RosMessageTypes.Geometry.TwistMsg>("/robot_twist");
        ros.RegisterPublisher<RosMessageTypes.Geometry.TwistMsg>("/robot_camera_twist");
    }

    void Update()
    {
        if (originObserver.TargetStatus.Status == Status.TRACKED &&
            destinationObserver.TargetStatus.Status == Status.TRACKED)
        {
            UpdateLine();
            //Verificar si hay que multiplicar por -1
            float desiredTwist = Vector3.SignedAngle(markerOrigin.transform.forward, markerDestination.transform.position- markerOrigin.transform.position, new Vector3(0,1,0)); 
            twist.linear.x = 0.5;
            //Verificar si es Z o Y
            twist.angular.z = desiredTwist; 
            ros.Publish("/robot_twist", twist);
        }
        else
        {
            lineRenderer.enabled = false;
            //Send empty Twist msg
            twist.linear.x = 0;
            //Verificar si es Z o Y
            twist.angular.z = 0;
            ros.Publish("/robot_twist", twist);
        }
    }

    private void OnObserverStatusChanged(ObserverBehaviour behaviour, TargetStatus status)
    {
        if (status.Status == Status.TRACKED)
        {
            lineRenderer.enabled = true;
        }
        else if (originObserver.TargetStatus.Status != Status.TRACKED ||
                 destinationObserver.TargetStatus.Status != Status.TRACKED)
        {
            lineRenderer.enabled = false;
        }
    }

    private void UpdateLine()
    {
        lineRenderer.SetPosition(0, markerOrigin.transform.position);
        lineRenderer.SetPosition(1, markerDestination.transform.position);
    }
}
