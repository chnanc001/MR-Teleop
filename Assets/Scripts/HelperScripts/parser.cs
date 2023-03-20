// written by Guanhao Fu, 2020﻿

using UnityEngine;
using System.Text.RegularExpressions;
using System;
using System.IO;
using System.Globalization;
using MathNet.Numerics.LinearAlgebra;

public class parser
{

    // convert json msg string to Matrix4x4
    public static Matrix4x4 GetMatrix4X4(string file)
    {
        string json = JsonHelper.GetPosition(file, "Position");
        //string json = JsonHelper.GetSetPointPosition(file, "setpiont_cp");
        string Position = JsonHelper.GetTranslation(json, "Translation");
        string Rotation = JsonHelper.GetRotation(json, "Rotation");
        Vector3 Pos = StringToVector3(Position);
        Matrix4x4 m = new Matrix4x4();
        m = StringToRot(Rotation);
        m.SetColumn(3, Pos);
        m.SetRow(3, new Vector4(0, 0, 0, 1));
        return m;
    }


    // return translation only
    public static Vector3 GetPos(string file)
    {
        string json = JsonHelper.GetPosition(file, "Position");
        string Position = JsonHelper.GetTranslation(json, "Translation");

        Vector3 Pos = StringToVector3(Position);

        return Pos;
    }

    public static Vector3 StringToVector3(string sVector)
    {
        // Remove the brackets
        sVector = sVector.Replace("[", string.Empty);
        sVector = sVector.Replace("]", string.Empty);

        // split the items
        string[] sArray = sVector.Split(',');

        // Switch y and z translation since dVRK has diff coordinate.
        Vector3 result = new Vector3(
            float.Parse(sArray[0]),
            float.Parse(sArray[2]),
            float.Parse(sArray[1]));

        return result;
    }

    public static Matrix4x4 StringToRot(string sRot)
    {
        // Remove the brackets
        sRot = sRot.Replace("[", string.Empty);
        sRot = sRot.Replace("]", string.Empty);

        // split the items
        string[] sArray = sRot.Split(',');

        Matrix4x4 m = new Matrix4x4();
        // store as a Vector3
        Vector3 row1 = new Vector3(
            float.Parse(sArray[0]),
            float.Parse(sArray[1]),
            float.Parse(sArray[2]));

        Vector3 row2 = new Vector3(
            float.Parse(sArray[3]),
            float.Parse(sArray[4]),
            float.Parse(sArray[5]));

        Vector3 row3 = new Vector3(
            float.Parse(sArray[6]),
            float.Parse(sArray[7]),
            float.Parse(sArray[8]));

        m.SetRow(0, row1);
        m.SetRow(1, row2);
        m.SetRow(2, row3);

        Vector3 offset1 = new Vector3(1, 0, 0);
        Vector3 offset2 = new Vector3(0, 0, 1);
        Vector3 offset3 = new Vector3(0, 1, 0);

        Matrix4x4 offset = new Matrix4x4();
        offset.SetRow(0, offset1);
        offset.SetRow(1, offset2);
        offset.SetRow(2, offset3);

        // orignial had below line unedited
        //m = offset * m;
        return m;
    }


    public static bool StringMatch(string dVRK_msg, string pattern)
    {
        Regex regx = new Regex(pattern);
        Match match = regx.Match(dVRK_msg);
        return match.Success;
    }

    public static float GetJawAngle(string dVRK_msg)
    {
        // extract jaw angle
        string pattern = "\"Position\":";
        float jaw_angle = float.Parse(JsonHelper.JawAngle(dVRK_msg, pattern));
        return jaw_angle;
    }

    public static Vector<float> GetJoints(string dVRK_msg)
    {
        string pattern = "\"Position\":";
        string allJoints = JsonHelper.Joints(dVRK_msg, pattern);
        string[] joints_array = allJoints.Split(',');
        Vector<float> joints = Vector<float>.Build.Random(6);
        for (int i = 0; i < 6; i++)
        {
            joints[i] = float.Parse(joints_array[i], CultureInfo.InvariantCulture.NumberFormat);
        }
        return joints;
    }

}
