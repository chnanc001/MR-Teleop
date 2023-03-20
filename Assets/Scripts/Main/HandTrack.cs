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

public class HandTrack : MonoBehaviour
{

    // declare variables //

    // for hand tracking
    public GameObject sphere_marker;
    private GameObject index;
    private GameObject thumb;
    private GameObject thumb_prox;
    private GameObject middle;
    public static Vector3 index_pos;
    public static Vector3 thumb_pos;
    Vector3 thumb_prox_pos;
    public static Vector3 middle_pos;
    Vector3 wrist_pos;
    Quaternion wrist_rot;
    MixedRealityPose pose;
    Quaternion thumb_rot;

    // for teleop control
    // public Text hand_warn;
    bool first_pinch = true;
    Vector3 hand_start_pos;
    Quaternion hand_start_rot;
    GameObject EE_virt_pos;
    bool line_follow_start = false;
    Vector3 EE_virt_pos_start;
    Vector3 EE_line_start_pos;
    bool line_follow = false;
    Quaternion EE_line_start_rot;

    // for jaw angle control
    float angle_des;     // desired jaw angle from hand tracking
    float jaw_angle;     // read in jaw angle from dVRK
    float send_angle;    // jaw angle to send 
    float T = 0.0f;
    bool angle_adj_stop = false;

    // for pose control
    Vector3 EE_pos;        // EE pos from dVRK
    Quaternion EE_quat;    // EE rot from dVRK
    public static Vector3 new_EE_pos;
    Quaternion new_EE_rot;
    private Vector3 new_EE_pos_send;
    Vector3 EE_start_pos;
    Quaternion EE_start_rot;
    float scale = 0.2f;    // translational scaling factor
    Quaternion thumb_prox_rot;
    Quaternion scaled_new_EE_rot;
    Matrix<float> new_EE_rot_send = Matrix<float>.Build.Random(3, 3);
    Quaternion holo_2_endo = new Quaternion(0, 1, 0, 0);    // transform from holo coord to dVRK endo coord
    Quaternion hand_rot_change;    // change in hand orientation

    // for udp message sending
    string pose_message;
    string jaw_message;






    public GameObject hololens;    // hololens

    float dist2dVRK;


    public static bool teleop;

    Quaternion index_rot;



    public GameObject UDP;    // udp comm 


    public TextMesh teleop_text;
    public TextMesh plan_text;

    //public GameObject qr_tracker;




    public static float pinch_dist;
    private Queue<float> pinch_dist_q = new Queue<float>();


    public GameObject axis;
    GameObject hand_axis;

    Quaternion start_axis;


    public GameObject teleop_on;
    public GameObject teleop_off;


    bool teleop_lock = false;


    // Start is called before the first frame update
    void Start()
    {
        // create spheres object to attach to fingers
        //index = Instantiate(sphere_marker, this.transform);
        thumb = Instantiate(sphere_marker, this.transform);
        //thumb_prox = Instantiate(sphere_marker, this.transform);
        //middle = Instantiate(sphere_marker, this.transform);

        hand_axis = Instantiate(axis, this.transform);
        hand_axis.transform.localScale = new Vector3(0.025f, 0.025f, 0.025f);

    }

    // Update is called once per frame
    void Update()
    {

        // get hand tracking positions
        // spheres only visible if hand is visible
        //index.GetComponent<Renderer>().enabled = false;
        thumb.GetComponent<Renderer>().enabled = false;
        //thumb_prox.GetComponent<Renderer>().enabled = false;
        //middle.GetComponent<Renderer>().enabled = false;

        hand_axis.SetActive(false);

        // get index pos
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, Handedness.Right, out pose))
        {
            index_pos = pose.Position;
            index_rot = pose.Rotation;
            //index.GetComponent<Renderer>().enabled = true;
            //index.transform.position = pose.Position;
        }
        // get thumb pos
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, Handedness.Right, out pose))
        {
            thumb_pos = pose.Position;
            thumb_rot = pose.Rotation;
            thumb.GetComponent<Renderer>().enabled = true;
            thumb.transform.position = pose.Position;
        }
        // get thumb prox pos
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbProximalJoint, Handedness.Right, out pose))
        {
            thumb_prox_pos = pose.Position;
            thumb_prox_rot = pose.Rotation;
            //thumb_prox.GetComponent<Renderer>().enabled = true;
            //thumb_prox.transform.position = pose.Position;

            // rotation axis
            hand_axis.SetActive(true);
            hand_axis.transform.position = pose.Position;
            hand_axis.transform.rotation = pose.Rotation;

        }
        // get middle pos
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.MiddleTip, Handedness.Right, out pose))
        {
            middle_pos = pose.Position;
            //middle.GetComponent<Renderer>().enabled = true;
            //middle.transform.position = pose.Position;
        }
        // get wrist pos
        //if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Right, out pose))
        //{
        //    wrist_pos = pose.Position;
        //    wrist_rot = pose.Rotation;
        //}


        // get dVRK pose and jaw from UDP
        jaw_angle = UDPComm.jaw_angle;
        EE_pos = UDPComm.EE_pos;
        EE_quat = UDPComm.EE_quat;

        Debug.Log("EE pos: " + EE_pos);

        pinch_dist = Vector3.Magnitude(index_pos - thumb_pos);
        // use moving average of pinch distance
        // add elements to queue
        //if (pinch_dist_q.Count < 1)
        //{
        //    pinch_dist_q.Enqueue(pinch_dist);
        //}
        //else
        //{
        //    pinch_dist_q.Dequeue();
        //    pinch_dist_q.Enqueue(pinch_dist);
        //}

        //Quaternion EE_home_quat = new Quaternion(0.70711f, 0.70711f, 0.0f, 0.0f);

        teleop_text.GetComponent<Renderer>().enabled = false;
        teleop_off.SetActive(false);
        teleop_on.SetActive(false);

       // teleop = true;
        //Calib.pinch_calibration_complete = true;
        if (teleop & Calib.pinch_calibration_complete)
        {

            teleop_off.SetActive(true);
            teleop_on.SetActive(false);
            // visual feedback for teleop mode
            teleop_text.GetComponent<Renderer>().enabled = true;
            //teleop_text.transform.position = qr_tracker.transform.position + new Vector3(0.0f, 0.15f, 0.0f);

            // if hand is in view
            if (thumb.GetComponent<Renderer>().enabled)
            {

                //Debug.Log("pinch dist: " + pinch_dist);
               // Debug.Log("calibrated dist: " + Calib.calibrated_pinch);
                Debug.Log("Should be in the 2nd if loop");
                // if index+thumb == pinching -> can teleoperate
                //if (Vector3.Magnitude(index_pos - thumb_pos) < 0.02)
                if (pinch_dist < Calib.calibrated_pinch)
                {
                    teleop_lock = true;
                }
                else if (pinch_dist > (Calib.calibrated_pinch + 0.011))    // only stop teleop if pinch dist greater than certain threshold
                {
                    teleop_lock = false;
                }

                    Debug.Log(teleop_on);


                if (teleop_lock)
                {

                    teleop_off.SetActive(false);
                    teleop_on.SetActive(true);
                    //Debug.Log("in the third if loop");

                    if (first_pinch)
                    {
                        first_pinch = false;

                        // get starting hand position
                        hand_start_pos = hololens.transform.position + thumb_prox_pos;
                        //hand_start_rot = hololens.transform.rotation * index_rot;

                        // save start position of EE
                        if (new_EE_pos == Vector3.zero | EE_pos == new_EE_pos)
                        {
                            EE_start_pos = EE_pos;
                            EE_start_rot = EE_quat;
                        }
                        else
                        {
                            // this is a hack, NEEDS to be changed!!
                            EE_start_pos = new_EE_pos;
                            EE_start_rot = new_EE_rot;
                        }

                        start_axis = hand_axis.transform.rotation;

                    }


                    // write pose message //

                    // translation
                    // get new EE position as scaled of how much hand has moved, relative translation
                    new_EE_pos = EE_start_pos + scale * Vector3.Scale(((hololens.transform.position + thumb_prox_pos) - hand_start_pos), new Vector3(-1.0f, 1.0f, -1.0f));

                    // convert pos to same orientation as dVRK i.e. y and z swapped
                    new_EE_pos_send[0] = new_EE_pos[0];
                    new_EE_pos_send[1] = new_EE_pos[2];
                    new_EE_pos_send[2] = new_EE_pos[1];

                    // rotation
                    // get new rot of EE based off hand rot, relative rotation (not absolute)

                    // get change in hand rotation
                    //hand_rot_change = (Quaternion.Inverse(Quaternion.Inverse(holo_2_endo) * hand_start_rot) * Quaternion.Inverse(holo_2_endo) * hololens.transform.rotation * index_rot);

                    //hand_rot_change = new(hand_rot_change.x, hand_rot_change.z, hand_rot_change.y, hand_rot_change.w);


                    // get new EE orientation

                    // NEW ROTATION ATTEMPT ////////////////
                    Quaternion axis_rot = (Quaternion.Inverse(start_axis) * hand_axis.transform.rotation);
                    //axis_rot = new(axis_rot.x, axis_rot.z, axis_rot.y, axis_rot.w);

                    //Quaternion offset = new(-0.7071f, 0.0f, 0.0f, 0.7071f);

                    //axis_rot = Quaternion.Inverse(offset) * axis_rot * offset;

                    new_EE_rot = axis_rot * EE_start_rot;
                    ////////////////////////////////////////

                    // old rotation
                    //new_EE_rot = EE_start_rot * hand_rot_change;

                    // scale quaternion
                    //scaled_new_EE_rot = Quaternion.Slerp(EE_start_rot, new_EE_rot, 1f);


                    // convert normalised quaternion to rotation matrix
                    new_EE_rot_send = Quat2Rot(Quaternion.Normalize(new_EE_rot));


                    // get message to send to dVRK (new EE pose)
                    pose_message = VectorFromMatrix(new_EE_pos_send, new_EE_rot_send);


                    // write jaw message //

                    //// desried jaw angle from hand
                    //angle_des = DesAngle(thumb_pos, middle_pos, thumb_prox_pos);



                    //// adjust jaw angle until jaw angle == finger angle
                    //if (Math.Abs(jaw_angle - angle_des) < 0.1)
                    //{
                    //    angle_adj_stop = true;
                    //}
                    //// if jaw angle caught up to finger angle, send desired angle
                    //if (angle_adj_stop)
                    //{
                    //    send_angle = (float)angle_des;
                    //}
                    //// if not, move jaws towards desired angle
                    //// current rate: jaw angle reach desired angle in 10s
                    //else
                    //{
                    //    send_angle = ((angle_des - jaw_angle) / 10.0f) * T + jaw_angle;
                    //    T += Time.deltaTime;
                    //}

                    send_angle = jaw_angle;

                    // jaw message to send to dVRK
                    jaw_message = "{\"jaw/servo_jp\":{\"Goal\":[" + send_angle.ToString("R") + "]}}";


                    // send json strings to dVRK 
                    UDP.GetComponent<UDPComm>().UDPsend(pose_message, jaw_message);
                }

                // not teleoperating
                else
                {
                    first_pinch = true;
                    angle_adj_stop = false;
                    T = 0.0f;
                }
            }
        }

        // if not in teleop mode
        else
        {
            // remove visual feedback
            teleop_text.GetComponent<Renderer>().enabled = false;
        }
    }




    // --- functions --- //

    // get desired jaw angle from middle and thumb positions
    // dVRK jaw -> [-20; 80] degrees, fingers -> [10; 50] degrees
    public float DesAngle(Vector3 thumb_pos, Vector3 middle_pos, Vector3 thumb_prox_pos)
    {
        // get length of thumb, index and thumb-index
        float thumb_len = (thumb_pos - thumb_prox_pos).sqrMagnitude;
        float middle_len = (middle_pos - thumb_prox_pos).sqrMagnitude;
        float thumb_index_len = (middle_pos - thumb_pos).sqrMagnitude;
        // angle formed by index finger and thumb
        double finger_angle = Mathf.Acos((thumb_len + middle_len - thumb_index_len) / (2 * Mathf.Sqrt(thumb_len) * Mathf.Sqrt(middle_len)));
        // desired jaw angle in radians (mapping: -20 jaw == 30 fingers, 80 jaw == 50 fingers)
        double angle_des = (5 * (Mathf.Rad2Deg * finger_angle) - 170);
        //    -20 <= angle_des <= 80
        if (angle_des < -20.0)
        {
            angle_des = -20.0;
        }
        else if (angle_des > 80.0)
        {
            angle_des = 80.0;
        }
        // return desired jaw angle
        return (float)angle_des * Mathf.Deg2Rad;
    }

    // get quaternion from homogeneous matrix
    public static Quaternion QuaternionFromMatrix(Matrix4x4 m)
    {
        return Quaternion.LookRotation(m.GetColumn(2), m.GetColumn(1));
    }

    // position vector and 3x3 rotation matrix to json string
    public string VectorFromMatrix(Vector3 pos, Matrix<float> rot)
    {
        jsonFloat jsonfloat = new jsonFloat();

        // translation
        float[] translation = new float[3];
        for (int i = 0; i < 3; i++) // only get the first 3 rows of Transformation matrix
        {
            translation[i] = pos[i];
        }

        // rotation
        string rotation = string.Empty;
        rotation += "[";
        for (int i = 0; i < 3; i++)
        {
            if (i == 0)
            {
                rotation += "[";
            }
            else
            {
                rotation += ",[";
            }
            for (int j = 0; j < 3; j++)
            {
                rotation += rot[i, j].ToString("R").ToLower();    // dVRK seems to only read lower case e for scientific notation (i.e. e-07 and not E-07)
                if (j == 2)
                {
                    rotation += "]";
                }
                else
                {
                    rotation += ",";
                }
            }
        }

        rotation += "]";
        jsonfloat.Rotation = rotation;
        jsonfloat.Translation = translation;
        string mockString = JsonUtility.ToJson(jsonfloat);
        //begin funny string operation because jsonUtility doesn't support 2d array serialization
        string[] mockArray = mockString.Split('[');
        string first_part = mockArray[0].Substring(0, mockArray[0].Length - 1);
        string second_part = mockString.Substring(mockArray[0].Length, mockString.Length - mockArray[0].Length);
        string[] mockArray_2 = second_part.Split('\"');
        string second_part_2 = mockArray_2[0];
        string third_part = second_part.Substring(second_part_2.Length + 1, second_part.Length - (second_part_2.Length + 1));
        string final_string = first_part + second_part_2 + third_part;
        final_string = "{\"servo_cp\": {\"Goal\": " + final_string + "}}";
        return final_string;
    }

    // convert quaternion to rotation matrix
    public Matrix<float> Quat2Rot(Quaternion q)
    {
        Matrix<float> m = Matrix<float>.Build.Random(3, 3);

        m[0, 0] = 1 - 2 * Mathf.Pow(q.y, 2) - 2 * Mathf.Pow(q.z, 2);
        m[0, 1] = 2 * q.x * q.y - 2 * q.w * q.z;
        m[0, 2] = 2 * q.x * q.z + 2 * q.w * q.y;
        m[1, 0] = 2 * q.x * q.y + 2 * q.w * q.z;
        m[1, 1] = 1 - 2 * Mathf.Pow(q.x, 2) - 2 * Mathf.Pow(q.z, 2);
        m[1, 2] = 2 * q.y * q.z - 2 * q.w * q.x;
        m[2, 0] = 2 * q.x * q.z - 2 * q.w * q.y;
        m[2, 1] = 2 * q.y * q.z + 2 * q.w * q.x;
        m[2, 2] = 1 - 2 * Mathf.Pow(q.x, 2) - 2 * Mathf.Pow(q.y, 2);

        return m;
    }


    // moving average
    private float MovingAveragePos(Queue<float> pinch_dist_q)
    {
        float sum = 0.0f;


        for (int i = 0; i < pinch_dist_q.Count; i++)
        {
            sum += pinch_dist_q.ElementAt(i);
        }

        return sum / pinch_dist_q.Count;

    }

}

