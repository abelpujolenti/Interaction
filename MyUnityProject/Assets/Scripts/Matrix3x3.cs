using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Matrix3x3
{
    public float[][] values;
    public Matrix3x3(float v) {
        values = new float[3][];
        for (int i = 0; i < 3; i++)
        {
            values[i] = new float[3];
            for (int j = 0; j < 3; j++)
            {
                if (i == j) values[i][j] = v;
                else values[i][j] = 0f;
            }
        }
    }

    public Vector3 MultiplyByVector3(Vector3 v)
    {
        return new Vector3(
            values[0][0] * v.x + values[0][1] * v.y + values[0][2] * v.z,
            values[1][0] * v.x + values[1][1] * v.y + values[1][2] * v.z,
            values[2][0] * v.x + values[2][1] * v.y + values[2][2] * v.z
            );
    }
}
