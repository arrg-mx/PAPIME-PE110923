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
    public GameObject zArrow;
    public GameObject xArrow;
    public GameObject xpArrow;
    public float scaleFactor;
    private ObserverBehaviour originObserver;
    private ObserverBehaviour destinationObserver;
    //ROS
    public ROSConnection ros;
    RosMessageTypes.Geometry.TwistMsg twist = new();
    RosMessageTypes.Geometry.TwistMsg twistCam = new();
    //RosMessageTypes.Std.Float32MultiArrayMsg us = new();
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
    
        ros.RegisterPublisher<RosMessageTypes.Std.StringMsg>("/topic2");
        ros.RegisterPublisher<RosMessageTypes.Geometry.TwistMsg>("/robot_twist");
        ros.RegisterPublisher<RosMessageTypes.Geometry.TwistMsg>("/robot_camera_twist");
        ros.Subscribe<RosMessageTypes.Std.Float32MultiArrayMsg>("/robot_us", ArrowScale);
    }

    void Update()
    {
        if (originObserver.TargetStatus.Status == Status.TRACKED &&
            destinationObserver.TargetStatus.Status == Status.TRACKED)
        {
            UpdateLine();
            Vector3 distanceVector = markerDestination.transform.position - markerOrigin.transform.position;
            float desiredTwist = -Vector3.SignedAngle(markerOrigin.transform.forward, distanceVector, new Vector3(0,1,0));
            float desiredLinear = (distanceVector.magnitude - 1.7f);
            Debug.Log("desiredLinearPrev = " + desiredLinear);
            desiredLinear = Mathf.Clamp(desiredLinear, 0.0f, 1.0f);
            if (desiredTwist < 5f && desiredTwist >= 0f) desiredTwist = 0f;
            if (desiredTwist > -5f && desiredTwist <= 0f) desiredTwist = 0f;
            twist.linear.x = desiredLinear;
            twist.angular.z = desiredTwist;
            Debug.Log("Dist = " + distanceVector.magnitude);
            Debug.Log("Lin = " + desiredLinear);
            Debug.Log("Ang = " + desiredTwist);
            ros.Publish("/robot_twist", twist);
        }
        else
        {
            lineRenderer.enabled = false;
            //Send empty Twist msg
            twist.linear.x = 0;
            twist.angular.z = 0;
            ros.Publish("/robot_twist", twist);
        }
    }
    private void OnApplicationQuit()
    {
        twist = new();
        ros.Publish("/robot_twist", twist);
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

    void ArrowScale(RosMessageTypes.Std.Float32MultiArrayMsg us)
    {

        if (us.data[0] > 20f)
        {
            xpArrow.transform.localScale = new Vector3(20f * scaleFactor, 20f * scaleFactor, 20f * scaleFactor);
        }
        else 
        {
            xpArrow.transform.localScale = new Vector3(us.data[0] * scaleFactor, us.data[0] * scaleFactor, us.data[0] * scaleFactor);
        }
        if (us.data[1] > 20f)
        {
            xArrow.transform.localScale = new Vector3(20f * scaleFactor, 20f * scaleFactor, 20f * scaleFactor);
        }
        else 
        {
            xArrow.transform.localScale = new Vector3(us.data[1] * scaleFactor, us.data[1] * scaleFactor, us.data[1] * scaleFactor);
        }
        if (us.data[2] > 20f)
        {
            zArrow.transform.localScale = new Vector3(20f * scaleFactor, 20f * scaleFactor, 20f * scaleFactor);
        }
        else 
        {
            zArrow.transform.localScale = new Vector3(us.data[2] * scaleFactor, us.data[2] * scaleFactor, us.data[2] * scaleFactor);
        }

    }

    private void UpdateLine()
    {
        lineRenderer.SetPosition(0, markerOrigin.transform.position);
        lineRenderer.SetPosition(1, markerDestination.transform.position);
    }
}
