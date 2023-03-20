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

public class Main2 : MonoBehaviour
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

    bool teleop = false;

    Quaternion index_rot;

    public GameObject UDP;    // udp comm 


    public TextMesh teleop_text;
    public TextMesh plan_text;

    float pinch_dist;
    private Queue<float> pinch_dist_q = new Queue<float>();

    int pinch_counter = 0;
    bool pinch_calibration_complete = false;
    bool pinch_calibration_ongoing = false;
    float calibration_timer = 0;
    float[] calibration_pinch = new float[5];
    float calibrated_pinch;
    float counter = 0;
    float timer_hold;

    bool isrunning = false;
    public TextMesh pinch; 
    // Start is called before the first frame update
    void Start()
    {
        // create spheres object to attach to fingers
        index = Instantiate(sphere_marker, this.transform);
        thumb = Instantiate(sphere_marker, this.transform);
        thumb_prox = Instantiate(sphere_marker, this.transform);
        middle = Instantiate(sphere_marker, this.transform);
    }

    // Update is called once per frame
    void Update()
    {
        // get hand tracking positions
        // spheres only visible if hand is visible
        index.GetComponent<Renderer>().enabled = false;
        thumb.GetComponent<Renderer>().enabled = false;
        thumb_prox.GetComponent<Renderer>().enabled = false;
        middle.GetComponent<Renderer>().enabled = false;

        // get index pos
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, Handedness.Right, out pose))
        {
            index_pos = pose.Position;
            index_rot = pose.Rotation;
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
            thumb_prox_pos = pose.Position;
            thumb_prox_rot = pose.Rotation;
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
            wrist_pos = pose.Position;
            wrist_rot = pose.Rotation;
        }


        // get dVRK pose and jaw from UDP
        jaw_angle = UDPComm.jaw_angle;
        EE_pos = UDPComm.EE_pos;
        EE_quat = UDPComm.EE_quat;
  
         //M.Hadi
       pinch_dist = Vector3.Magnitude(index_pos - thumb_pos);
       
        if (pinch_counter > 0 & pinch_counter <6)
        {
            StopAllCoroutines();
            if (pinch_calibration_ongoing)
            {
                 while_pinch();
                
                StartCoroutine(while_pinch());
                // timer_hold = Time.deltaTime;
                //Debug.Log(calibration_timer);
                
                // ;
            }
           
            calibrated_pinch = (calibration_pinch[0] + calibration_pinch[1] + calibration_pinch[2] + calibration_pinch[3] + calibration_pinch[4]) / pinch_counter;
            //Debug.Log("calibrated_pinch: " + calibrated_pinch);
        }
        if (pinch_counter <= 5)
        {
            pinch.text = calibrated_pinch.ToString();
        }
        else
        {
            pinch.text = "Done Calibrating";

        }
        
        pinch.GetComponent<Renderer>().enabled = true;


        
            //M.Hadi

        // take moving average of pinch distance
        if (pinch_dist_q.Count < 5)
        {
            pinch_dist_q.Enqueue(pinch_dist);
        }
        else
        {
            pinch_dist_q.Dequeue();
            pinch_dist_q.Enqueue(pinch_dist);
        }


        //Debug.Log("EE rot: " + EE_quat.eulerAngles);
        //Debug.Log("angle: " + scaled_new_EE_rot.eulerAngles);
        //Debug.Log("hand rot: " + hand_rot_change);



        teleop = true;
        if (teleop & pinch_calibration_complete)
        {
            // teleop //

            // if hand is in view
            if (thumb_prox.GetComponent<Renderer>().enabled)
            {
                // if index+thumb == pinching -> can teleoperate
                //if (Vector3.Magnitude(index_pos - thumb_pos) < 0.02)
               // Debug.Log("Averaged pinch dist: " + MovingAveragePos(pinch_dist_q));
                if (MovingAveragePos(pinch_dist_q) < calibrated_pinch)
                {

                    if (first_pinch)
                    {
                        first_pinch = false;

                        // get starting hand position
                        hand_start_pos = hololens.transform.position + index_pos;
                        hand_start_rot = hololens.transform.rotation * index_rot;

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
                            EE_start_rot = scaled_new_EE_rot;
                            //jaw_angle = send_angle;
                        }
                    }


                    // write pose message //

                    // translation
                    // get new EE position as scaled of how much hand has moved, relative translation
                    new_EE_pos = EE_start_pos + scale * Vector3.Scale(((hololens.transform.position + index_pos) - hand_start_pos), new Vector3(-1.0f, 1.0f, -1.0f));

                    // convert pos to same orientation as dVRK i.e. y and z swapped
                    new_EE_pos_send[0] = new_EE_pos[0];
                    new_EE_pos_send[1] = new_EE_pos[2];
                    new_EE_pos_send[2] = new_EE_pos[1];

                    // rotation
                    // get new rot of EE based off hand rot, relative rotation (not absolute)

                    // get change in hand rotation
                    hand_rot_change = (Quaternion.Inverse(Quaternion.Inverse(holo_2_endo) * hand_start_rot) * Quaternion.Inverse(holo_2_endo) * hololens.transform.rotation * index_rot);

                    // get new EE orientation
                    new_EE_rot = hand_rot_change * EE_start_rot;

                    // scale quaternion
                    scaled_new_EE_rot = Quaternion.Slerp(EE_start_rot, new_EE_rot, 1f);



                    // convert normalised quaternion to rotation matrix
                    new_EE_rot_send = Quat2Rot(Quaternion.Normalize(scaled_new_EE_rot));

                    // get message to send to dVRK (new EE pose)
                    pose_message = VectorFromMatrix(new_EE_pos_send, new_EE_rot_send);


                    // write jaw message //

                    // desried jaw angle from hand
                    angle_des = DesAngle(thumb_pos, middle_pos, thumb_prox_pos);

                    // adjust jaw angle until jaw angle == finger angle
                    if (Math.Abs(jaw_angle - angle_des) < 0.1)
                    {
                        angle_adj_stop = true;
                    }
                    // if jaw angle caught up to finger angle, send desired angle
                    if (angle_adj_stop)
                    {
                        send_angle = (float)angle_des;
                    }
                    // if not, move jaws towards desired angle
                    // current rate: jaw angle reach desired angle in 10s
                    else
                    {
                        send_angle = ((angle_des - jaw_angle) / 10.0f) * T + jaw_angle;
                        T += Time.deltaTime;
                    }

                    // jaw message to send to dVRK
                    jaw_message = "{\"jaw/servo_jp\":{\"Goal\":[" + send_angle.ToString("R") + "]}}";


                    // send json strings to dVRK 
                    //UDP.GetComponent<UDPComm>().UDPsend(pose_message, jaw_message);
                }

                // index+thumb not pinching
                else
                {
                    first_pinch = true;
                    angle_adj_stop = false;
                    T = 0.0f;
                }
            }
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
        // desired jaw angle in radians (mapping: 1.25*finger_angle-20  <- in degrees)
        double angle_des = (2.5f * (Mathf.Rad2Deg * finger_angle) - 45.0f);
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



    public void pinch_calibration()

    {
        pinch_counter++;
        pinch_calibration_ongoing = true;
        Debug.Log("pinch_counter: " + pinch_counter);
        if (pinch_counter > 5)
        {
            pinch_calibration_complete = true;
            pinch_counter = 0;
            Debug.Log("Done calibrating");
        }

    }

    IEnumerator while_pinch()

    {
         //if(isrunning) yield brake;
           // isrunning = true;
        while (calibration_timer < 5.0f)
           
        //while(counter<5000)  
        {
            yield return new WaitForSeconds(2); 
                    calibration_pinch[pinch_counter - 1] += pinch_dist;
                    calibration_timer += Time.deltaTime;
                    counter++;
           
           
            //yield return new WaitForSeconds(.1f);
            //Debug.Log(counter); 
        }
       // isrunning = false;
        calibration_timer = 0;
        //Debug.Log(calibration_pinch[pinch_counter - 1]);
        calibration_pinch[pinch_counter - 1] /= counter;
        counter = 0;
        pinch_calibration_ongoing = false;
        //Debug.Log(pinch_calibration_ongoing);
        
    }

}

