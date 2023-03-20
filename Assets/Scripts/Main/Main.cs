
// main script for MR Hololens dVRK teleop

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.Input;

public class Main : MonoBehaviour
{

   public static bool MTM = false;

    float fist_timer = 0.0f;
    float handopen_timer = 0.0f;

    // visual for MTM control
    public TextMesh MTM_text;

    public GameObject qr_tracker;

    // Start is called before the first frame update
    void Start()
    {
        // start in planning mode
        LineDrawer.line_draw = true;
        HandTrack.teleop = false;

        MTM_text.GetComponent<MeshRenderer>().enabled = false;
    }

    // Update is called once per frame
    void Update()
    {

        // MTM being used
        if (MTM)
        {
            MTM_text.GetComponent<MeshRenderer>().enabled = true;
            //MTM_text.transform.position = qr_tracker.transform.position + new Vector3(0.0f, 0.15f, 0.0f);
            LineDrawer.line_draw = false;
            HandTrack.teleop = false;
        }
        else
        {
            MTM_text.GetComponent<MeshRenderer>().enabled = false;
        }



        // when to enter teleop mode
        // only after hand is close for > 1.5 seconds
        if (Fist() && !MTM)
        {
            fist_timer += Time.deltaTime;
        }
        else
        {
            fist_timer = 0.0f;
        }
        // teleop
        if (fist_timer > 1.5f)
        {
            MTM_text.GetComponent<MeshRenderer>().enabled = false;
            HandTrack.teleop = true;
            LineDrawer.line_draw = false;
            fist_timer = 0.0f;
        }


        // when to enter planning mode
        // only after hand is open for > 2 seconds
        if (HandOpen() && !MTM)
        {
            handopen_timer += Time.deltaTime;
        }
        else
        {
            handopen_timer = 0.0f;
        }
        // plan
        if (handopen_timer > 2.0f)
        {
            MTM_text.GetComponent<MeshRenderer>().enabled = false;
            HandTrack.teleop = false;
            LineDrawer.line_draw = true;
            handopen_timer = 0.0f;
        }

    }



    private bool Fist()
    {
        bool pinky_curl = HandPoseUtils.PinkyFingerCurl(Handedness.Right) > 0.7;
        bool ring_curl = HandPoseUtils.RingFingerCurl(Handedness.Right) > 0.7;
        bool middle_curl = HandPoseUtils.MiddleFingerCurl(Handedness.Right) > 0.7;
        bool index_curl = HandPoseUtils.IndexFingerCurl(Handedness.Right) > 0.7;

        return pinky_curl && ring_curl && middle_curl && index_curl;
    }

    private bool HandOpen()
    {
        // when hand not in view Curl return 0 -> have min threshold (as well as the max threshold)
        bool pinky_curl = HandPoseUtils.PinkyFingerCurl(Handedness.Right) < 0.07 && HandPoseUtils.PinkyFingerCurl(Handedness.Right) > 0.0005;
        bool ring_curl = HandPoseUtils.RingFingerCurl(Handedness.Right) < 0.07 && HandPoseUtils.RingFingerCurl(Handedness.Right) > 0.0005;
        bool middle_curl = HandPoseUtils.MiddleFingerCurl(Handedness.Right) < 0.07 && HandPoseUtils.MiddleFingerCurl(Handedness.Right) > 0.0005;
        bool index_curl = HandPoseUtils.IndexFingerCurl(Handedness.Right) < 0.07 && HandPoseUtils.IndexFingerCurl(Handedness.Right) > 0.0005;

        return pinky_curl && ring_curl && middle_curl && index_curl;
    }

    // when MTM is being used to control dVRK
    // do NOT want to teleop with hand control (don't want to send UDP messages to PSM)
    public void MTMToggle()
    {
        MTM = !MTM;
    }



}
