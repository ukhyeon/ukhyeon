using UnityEngine;
using System.Collections;

public enum ControlMode { Keyboard, ROS };

public class AGVController : MonoBehaviour
{
    public GameObject[] wheelModels; // 바퀴 모델 배열
    public WheelCollider[] wheelColliders; // 바퀴 콜라이더 배열
    public ControlMode mode = ControlMode.Keyboard;

    public float maxLinearSpeed = 2; // m/s
    public float maxRotationalSpeed = 1; // rad/s
    public float wheelRadius = 0.033f; // meters
    public float trackWidth = 0.288f; // meters Distance between tyres
    public float forceLimit = 10;
    public float damping = 10;

    private float rosLinear = 0f;
    private float rosAngular = 0f;

    void Start()
    {
        for (int i = 0; i < wheelColliders.Length; i++)
        {
            SetParameters(wheelColliders[i]);
        }
    }

    void FixedUpdate()
    {
        if (mode == ControlMode.Keyboard)
        {
            KeyBoardUpdate();
        }
        else if (mode == ControlMode.ROS)
        {
            ROSUpdate();
        }
    }

    private void SetParameters(WheelCollider wheel)
    {
        WheelFrictionCurve forwardFriction = wheel.forwardFriction;
        WheelFrictionCurve sidewaysFriction = wheel.sidewaysFriction;

        forwardFriction.stiffness = forceLimit;
        sidewaysFriction.stiffness = damping;

        wheel.forwardFriction = forwardFriction;
        wheel.sidewaysFriction = sidewaysFriction;
    }

    private void KeyBoardUpdate()
    {
        float moveDirection = Input.GetAxis("Vertical");
        float turnDirection = Input.GetAxis("Horizontal");

        float inputSpeed = moveDirection * maxLinearSpeed;
        float inputRotationSpeed = turnDirection * maxRotationalSpeed;

        RobotInput(inputSpeed, inputRotationSpeed);
    }

    private void ROSUpdate()
    {
        // ROS 메시지 업데이트 처리 (추후 필요에 따라 구현)
    }

    private void RobotInput(float speed, float rotSpeed)
    {
        float wheel1Rotation = (speed / wheelRadius);
        float wheel2Rotation = wheel1Rotation;
        float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);

        if (rotSpeed != 0)
        {
            wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 2)) * Mathf.Rad2Deg;
            wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 2)) * Mathf.Rad2Deg;
        }
        else
        {
            wheel1Rotation *= Mathf.Rad2Deg;
            wheel2Rotation *= Mathf.Rad2Deg;
        }

        for (int i = 0; i < wheelColliders.Length; i++)
        {
            if (i % 2 == 0)
            {
                SetSpeed(wheelColliders[i], wheel1Rotation);
            }
            else
            {
                SetSpeed(wheelColliders[i], wheel2Rotation);
            }
        }
    }

    private void SetSpeed(WheelCollider wheel, float wheelSpeed)
    {
        JointSpring spring = wheel.suspensionSpring;
        spring.targetPosition = wheelSpeed;
        wheel.suspensionSpring = spring;

        WheelHit hit;
        if (wheel.GetGroundHit(out hit))
        {
            wheel.motorTorque = wheelSpeed;
        }
    }

    public void OnDrawGizmos()
    {
        // 경로 시각화 처리 (필요에 따라 구현)
    }
}
