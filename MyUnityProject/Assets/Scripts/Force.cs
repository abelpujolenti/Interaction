using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Force
{
    public Vector3 position;
    public Vector3 vector;
    public Force(Vector3 position, Vector3 vector)
    {
        this.position = position;
        this.vector = vector;
    }
}
