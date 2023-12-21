using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RigidBodyState
{
    public List<Force> forces = new List<Force>();
    public float mass = 5f;
    public Vector3 linearMomentum = Vector3.zero;
    public Vector3 angularMomentum = Vector3.zero;
    public float linearDamping = 0.99f;
    public float angularDamping = 0.95f;
    public Vector3 gravity = new Vector3(0f, -30f, 0f);
    public float airDensity = 1.293f;
    public float radius = 1f;
    public bool update = false;

    public Vector3 magnusForce = Vector3.zero;

    public void UpdateState(Transform t, bool applyMagnus, float dt)
    {
        linearMomentum = UpdateLinearMomentum(applyMagnus, dt);
        angularMomentum = UpdateAngularMomentum(t, dt);
        forces.Clear();

        t.position = UpdatePosition(t, dt);
        t.rotation = UpdateRotation(t, dt);
    }

    private Quaternion UpdateRotation(Transform t, float dt)
    {
        Vector3 angularVelocity = CalculateAngularVelocity();
        Vector3 w = angularVelocity * dt;
        Quaternion q = Quaternion.AngleAxis(w.magnitude, w.normalized);
        return t.rotation * q;
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
    private Vector3 UpdatePosition(Transform t, float dt)
    {
        return t.position + (CalculateLinearVelocity()) * dt;
    }
    private Vector3 UpdateLinearMomentum(bool applyMagnus, float dt)
    {
        //external forces
        Vector3 force = Vector3.zero;
        foreach (Force f in forces)
        {
            force += f.vector;
        }

        //magnus
        if (applyMagnus)
        {
            var direction = Vector3.Cross(CalculateAngularVelocity(), CalculateLinearVelocity());
            var magnitude = 4 / 3f * Mathf.PI * airDensity * Mathf.Pow(radius, 3);
            magnusForce = direction * magnitude;
            force += magnusForce;
        }

        //gravity
        force += gravity;

        return (linearMomentum + force * dt) * linearDamping;
    }
    private Vector3 UpdateAngularMomentum(Transform t, float dt)
    {
        return (angularMomentum + CalculateTorque(t) * dt) * angularDamping;
    }
    private Vector3 CalculateTorque(Transform t)
    {
        Vector3 torque = Vector3.zero;
        foreach (Force f in forces)
        {
            Vector3 r = f.position - t.position;
            torque += Vector3.Cross(r, f.vector);
        }
        return torque;
    }

    public void ApplyForce(Force force)
    {
        forces.Add(force);
    }
}
