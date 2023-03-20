//from video at https://youtu.be/6C1NPy321Nk
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Sinewave : MonoBehaviour
{
    public LineRenderer myLineRenderer;
    public int points;
    public float amplitude = 1;
    public float frequency = 1;
    public Vector2 xLimits = new Vector2(0, 1);
    public float movementSpeed = 1;
    [Range(0, 2 * Mathf.PI)]
    public float radians;
    void Start()
    {
        myLineRenderer = GetComponent<LineRenderer>();
    }

    void Draw()
    {
        float xStart = xLimits.x;
        float Tau = 2 * Mathf.PI;
        float xFinish = xLimits.y;
        float x_1 = 0;

        myLineRenderer.positionCount = 2*points;
        for (int currentPoint = 0; currentPoint < points; currentPoint++)
        {
            float progress = (float)currentPoint / (points - 1);
            x_1 = Mathf.Lerp(xStart, xFinish, progress);

            float y = amplitude * Mathf.Sin((Tau * frequency * x_1)); // + (Time.timeSinceLevelLoad * movementSpeed)); 
            myLineRenderer.SetPosition(currentPoint, new Vector3(x_1, y, 0));
        }

        for (int currentPoint = 0; currentPoint < points; currentPoint++)
        {
            float progress = (float)currentPoint / (points - 1);
            float x_2 = Mathf.Lerp(xStart + xFinish, xFinish + xFinish, progress);

            float z = amplitude * Mathf.Sin((Tau * frequency * x_2)); // + (Time.timeSinceLevelLoad * movementSpeed)); 
            myLineRenderer.SetPosition(currentPoint + points, new Vector3(x_2, 0, z));
        }



        Debug.Log("curved line from script");
        Vector3[] crv_line_pts = new Vector3[myLineRenderer.positionCount];
        myLineRenderer.GetPositions(crv_line_pts);
        for (int i = 0; i < myLineRenderer.positionCount; i++)
        {
            Debug.Log(crv_line_pts[i]);
        }

    }

    void Update()
    {
        Draw();
    }
}