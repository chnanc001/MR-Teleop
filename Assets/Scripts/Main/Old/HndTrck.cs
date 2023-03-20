using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

using Microsoft;
using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.Input;

using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.RegularExpressions;





#if !UNITY_EDITOR
using Windows.Networking.Sockets;
using Windows.Networking.Connectivity;
using Windows.Networking;
#endif

public class HndTrck : MonoBehaviour
{


    // for hand tracking
    public GameObject sphere_marker;
    private GameObject index;
    private GameObject thumb;
    private GameObject thumb_prox;
    private GameObject middle;
    UnityEngine.Vector3 index_pos;
    UnityEngine.Vector3 thumb_pos;
    UnityEngine.Vector3 thumb_prox_pos;
    UnityEngine.Vector3 middle_pos;
    MixedRealityPose pose;

    public GameObject self_holo;
    public GameObject qrTracker;

    //public GameObject QRtracker;

    // Start is called before the first frame update
    void Start()
    {
        // attached spheres to fingers
        index = Instantiate(sphere_marker, this.transform);
        thumb = Instantiate(sphere_marker, this.transform);
        thumb_prox = Instantiate(sphere_marker, this.transform);
        middle = Instantiate(sphere_marker, this.transform);


    }

    // Update is called once per frame
    void Update()
    {




        // get index pos
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, Handedness.Right, out pose))
        {
            index_pos = pose.Position;
            index.GetComponent<Renderer>().enabled = true;
            index.transform.position = pose.Position;
        }
        // get thumb pos
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, Handedness.Right, out pose))
        {
            thumb_pos = pose.Position;
            thumb.GetComponent<Renderer>().enabled = true;
            thumb.transform.position = pose.Position;

        }
        // get thumb prox pos
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbProximalJoint, Handedness.Right, out pose))
        {
            //thumb_prox_pos = pose.Position;
            //thumb_prox_rot = pose.Rotation;
            thumb_prox.GetComponent<Renderer>().enabled = true;
            thumb_prox.transform.position = pose.Position;

        }
        // get middle pos
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.MiddleTip, Handedness.Right, out pose))
        {
            middle_pos = pose.Position;
            middle.GetComponent<Renderer>().enabled = true;
            middle.transform.position = pose.Position;
        }
        // get wrist pos
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Right, out pose))
        {
            thumb_prox_pos = pose.Position;
        }


        Debug.Log("Dist to QR: " + Vector3.Magnitude(self_holo.transform.position - qrTracker.transform.position));

    }




}
