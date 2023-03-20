using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO;

using Microsoft;
using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.Input;

using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.RegularExpressions;

using MathNet.Numerics.LinearAlgebra;
using System.Linq;

public class HandTrack_test : MonoBehaviour
{



    Vector3 EE_pos;        // EE pos from dVRK
    Quaternion EE_quat;    // EE rot from dVRK
    float jaw_angle;     // EE jaw angle from dVRK

    Quaternion dVRK_holo_transform;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {



        // get dVRK pose and jaw from UDP
        jaw_angle = UDPComm.jaw_angle;
        EE_pos = UDPComm.EE_pos;
        EE_quat = UDPComm.EE_quat;


        // just coz parser swaps y and z already
        EE_pos = new Vector3(EE_pos.x, EE_pos.z, EE_pos.y);


        // dVRK to holo
        dVRK_holo_transform = new Quaternion(0.0f, 1.0f, 0.0f, 0.0f);





    }


    // get and process dVRK end effector pose



}
