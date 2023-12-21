using System.Collections;
using System.Collections.Generic;
using UnityEditor.SceneManagement;
using UnityEngine;

public class MovingBall : MonoBehaviour
{
    [SerializeField]
    IK_tentacles _myOctopus;

    //movement speed in units per second
    [Range(-1.0f, 1.0f)]
    [SerializeField]
    private float _movementSpeed = 5f;

    List<Force> forces = new List<Force>();
    float mass = 5f;
    Vector3 linearMomentum = Vector3.zero;
    Vector3 angularMomentum = Vector3.zero;
    float linearDamping = 0.99f;
    float angularDamping = 0.99f;
    Vector3 gravity = new Vector3(0f, -30f, 0f);
    float airDensity = 1.293f;
    float radius;

    void Start()
    {
        radius = GetComponent<SphereCollider>().radius * 10f;
    }

    void FixedUpdate()
    {
        linearMomentum = UpdateLinearMomentum();
        angularMomentum = UpdateAngularMomentum();
        forces.Clear();

        transform.position = UpdatePosition();
        transform.rotation = UpdateRotation();
    }
    private Quaternion UpdateRotation()
    {
        Vector3 angularVelocity = CalculateAngularVelocity();
        Vector3 w = angularVelocity * Time.deltaTime;
        Quaternion q = Quaternion.AngleAxis(w.magnitude, w.normalized);
        return transform.rotation * q;
    }
    private Vector3 CalculateAngularVelocity()
    {
        //inverse moment of inertia tensor
        Matrix3x3 mat = new Matrix3x3(1f / (0.4f * radius * radius * mass));
        //w = Inv(I) * L
        return mat.MultiplyByVector3(angularMomentum);
    }
    private Vector3 CalculateLinearVelocity()
    {
        return linearMomentum / mass;
    }
    private Vector3 UpdatePosition()
    {
        return transform.position + (CalculateLinearVelocity()) * Time.fixedDeltaTime;
    }
    private Vector3 UpdateLinearMomentum()
    {
        //external forces
        Vector3 force = Vector3.zero;
        foreach(Force f in forces)
        {
            force += f.vector;
        }

        //magnus
        var direction = Vector3.Cross(CalculateAngularVelocity(), CalculateLinearVelocity());
        var magnitude = 4 / 3f * Mathf.PI * airDensity * Mathf.Pow(radius, 3);
        force += direction * magnitude;

        //gravity
        force += gravity;

        return (linearMomentum + force * Time.fixedDeltaTime) * linearDamping;
    }
    private Vector3 UpdateAngularMomentum()
    {
        return (angularMomentum + CalculateTorque() * Time.fixedDeltaTime) * angularDamping;
    }
    private Vector3 CalculateTorque()
    {
        Vector3 torque = Vector3.zero;
        foreach (Force f in forces)
        {
            Vector3 r = f.position - transform.position;
            torque += Vector3.Cross(r, f.vector);
        }
        return torque;
    }

    public void ApplyForce(Vector3 force, Vector3 position)
    {
        forces.Add(new Force(position, force));
    }

    private void OnCollisionEnter(Collision collision)
    {
        _myOctopus.NotifyShoot();
    }
}
