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

using MathNet.Numerics.LinearAlgebra;




#if !UNITY_EDITOR
using Windows.Networking.Sockets;
using Windows.Networking.Connectivity;
using Windows.Networking;
#endif


public class HndTrck_line : MonoBehaviour
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

    // for teleop control
   // public GameObject teleop_led;    // visual feedback
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
    float angle_des;    // desired jaw angle from hand tracking
    float jaw_angle;    // read in jaw angle from dVRK
    float send_angle;    // jaw angle to send 
    float T = 0.0f;
    bool angle_adj_stop = false;

    // for pose control
    Vector3 EE_pos;  // EE pos from dVRK
    Quaternion EE_quat;
    Vector3 new_EE_pos;
    Quaternion new_EE_rot;
    private Vector3 new_EE_pos_send;
    Vector3 EE_start_pos;
    Quaternion EE_start_rot;
    float scale = 0.2f;
    Quaternion thumb_prox_rot;
    Quaternion scaled_new_EE_rot;
    Matrix<float> new_EE_rot_send = Matrix<float>.Build.Random(3, 3);
    Quaternion rot_off;

    // for udp socket connection
    byte[] data;
    Socket socket;
    EndPoint remote;
    byte[] send_msg;
    string pose_message;
    string jaw_message;
    string dVRK_msg;
    bool jaw_match;
    int read_msg_count = 0;

    // for line drawing
    public GameObject line;
    public GameObject current_line;
    public LineRenderer line_renderer;
    public List<Vector3> fingerPositions;





    public GameObject self_holo;
    public GameObject qr_tracker;
    public GameObject confirm_button;
    float dist2dVRK;

    // torus
    //public GameObject torus;
    Vector3 torus_start_pos;
    Quaternion torus_start_rot;



    bool teleop = false;









    // Start is called before the first frame update
    void Start()
    {
        // to keep gaze tracking pointer on
        PointerUtils.SetGazePointerBehavior(PointerBehavior.AlwaysOn);

        // visual feedback for teleop
       // teleop_led.SetActive(false);
       // hand_warn.enabled = false;

        // attached spheres to fingers
        index = Instantiate(sphere_marker, this.transform);
        thumb = Instantiate(sphere_marker, this.transform);
        thumb_prox = Instantiate(sphere_marker, this.transform);
        middle = Instantiate(sphere_marker, this.transform);

        // udp using socket
        data = new byte[1024];
        IPEndPoint ip = new IPEndPoint(IPAddress.Any, 48051);
        socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        socket.Bind(ip);
        IPEndPoint sender = new IPEndPoint(IPAddress.Any, 0);
        remote = (EndPoint)(sender);

        // create line object for drawing
        current_line = Instantiate(line, Vector3.zero, Quaternion.identity);
        line_renderer = current_line.GetComponent<LineRenderer>();

        // create sphere for EE pos visualisation
        EE_virt_pos = Instantiate(sphere_marker, this.transform);








    }

    // Update is called once per frame
    void Update()
    {
        //teleop_led.SetActive(false);

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
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Right, out pose))
        {
            thumb_prox_pos = pose.Position;
            thumb_prox_rot = pose.Rotation;
        }



        // dVRK seems to send messages faster than hololens runs update loop (this causes delay in messages read by hololens)
        // therefore read in messages from dVRK more than once per update loop to get newest messages
        read_msg_count = 0;
        while (read_msg_count < 1)
        {
            // read in message from dVRK
            //data = new byte[1024];
            socket.ReceiveFrom(data, ref remote);
            dVRK_msg = Encoding.UTF8.GetString(data);
            // determine if jaw or pose message
            jaw_match = parser.StringMatch(dVRK_msg, "\"jaw/setpoint_js\":");
            // if jaw message
            if (jaw_match)
            {
                // extract jaw angle
                jaw_angle = parser.GetJawAngle(dVRK_msg);
            }
            // if pose message
            else
            {
                // extract rot and pos
                EE_quat = QuaternionFromMatrix(parser.GetMatrix4X4(dVRK_msg));
                EE_pos = parser.GetPos(dVRK_msg);
            }
            read_msg_count += 1;
        }






        // place visual feedback of EE on line
        if (line_follow)
        {

            if (line_follow_start)
            {
                EE_virt_pos.GetComponent<Renderer>().enabled = true;
                line_follow_start = false;
                EE_line_start_pos = EE_pos;
                EE_line_start_rot = EE_quat;
                EE_virt_pos_start = fingerPositions[0];



                // torus
                //torus_start_pos = torus.transform.position;
                //torus_start_pos = fingerPositions[0];
                //torus_start_rot = torus.transform.rotation;

            }

            EE_virt_pos.transform.position = EE_virt_pos_start + Vector3.Scale((EE_line_start_pos - EE_pos), new Vector3(1.0f, -1.0f, -1.0f));


            // torus
            //torus.transform.position = torus_start_pos + Vector3.Scale((EE_line_start_pos - EE_pos), new Vector3(1.0f, -1.0f, -1.0f));
            //torus.transform.rotation = torus_start_rot * (Quaternion.Inverse(EE_line_start_rot) * EE_quat);

        }


       
        // control mode via distance of self from dVRK
        if (confirm_button.activeSelf == false)    // i.e. the position of QR code (and dVRK) is confirmed
        {
            dist2dVRK = Vector3.Magnitude(self_holo.transform.position - qr_tracker.transform.position);

            if (dist2dVRK < 0.5f)
            {
                teleop = false;    // can only enter plan mode
            }
            else
            {
                teleop = true;    // can only enter teleop mode
            }


        }












        if (teleop)
        {
            // teleop //
            // if hand is in view
            if (thumb_prox.GetComponent<Renderer>().enabled)
            {

              //  hand_warn.enabled = false;

                // if index+thumb == pinching -> can teleoperate
                if (Vector3.Magnitude(index_pos - thumb_pos) < 0.02)
                {

                    // visual feedback
                 //   teleop_led.SetActive(true);


                    if (first_pinch)
                    {
                        first_pinch = false;

                        // get starting hand position
                        hand_start_pos = self_holo.transform.position + thumb_prox_pos;
                        hand_start_rot = self_holo.transform.rotation * thumb_prox_rot;

                        // save start position of EE
                        EE_start_pos = EE_pos;
                        EE_start_rot = EE_quat;

                        // starting orientation between wrist and EE
                        rot_off = Quaternion.Inverse(hand_start_rot) * EE_start_rot;

                        // torus
                    //    torus_start_pos = torus.transform.position;
                    //    torus_start_rot = torus.transform.rotation;
                    }


                    // write pose message //

                    // translation
                    // get new EE position as scaled of how much hand has moved, relative translation
                    new_EE_pos = EE_start_pos + scale * Vector3.Scale(((self_holo.transform.position + thumb_prox_pos) - hand_start_pos), new Vector3(-1.0f, 1.0f, -1.0f));

                    //new_EE_pos = EE_start_pos;
                    
                    // convert pos to same orientation as dVRK i.e. y and z swapped
                    new_EE_pos_send[0] = new_EE_pos[0];
                    new_EE_pos_send[1] = new_EE_pos[2];
                    new_EE_pos_send[2] = new_EE_pos[1];



                    // rotation
                    // get new rot of EE based off hand rot, relative rotation (not absolute)
                    new_EE_rot = EE_start_rot * (Quaternion.Inverse(hand_start_rot) * self_holo.transform.rotation * thumb_prox_rot);
                    //new_EE_rot = self_holo.transform.rotation * thumb_prox_rot * rot_off;
                    // new Quaternion(-0.7071f, 0.0f, 0.0f, 0.7071f)

                    // scale quaternion
                    scaled_new_EE_rot = Quaternion.Slerp(EE_start_rot, new_EE_rot, 0.5f);

                    //Quaternion res_rot = new Quaternion(0.0f, 0.0f, scaled_new_EE_rot[2], scaled_new_EE_rot[3]);


                    // convert normalised quaternion to rotation matrix
                    new_EE_rot_send = Quat2Rot(Quaternion.Normalize(scaled_new_EE_rot));
                    //new_EE_rot_send = Quat2Rot(Quaternion.Normalize(res_rot));


                    // get message to send to dVRK (new EE pose)
                    pose_message = VectorFromMatrix(new_EE_pos_send, new_EE_rot_send);



                    // torus
                 //   torus.transform.position = torus_start_pos + self_holo.transform.position + thumb_prox_pos - hand_start_pos;
                 //   torus.transform.rotation = torus_start_rot * (Quaternion.Inverse(hand_start_rot) * (self_holo.transform.rotation) * thumb_prox_rot);



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


                    // send json strings to dVRK //
                    send_msg = Encoding.UTF8.GetBytes(pose_message);
                    socket.SendTo(send_msg, remote);
                    send_msg = Encoding.UTF8.GetBytes(jaw_message);
                    socket.SendTo(send_msg, remote);


                }

                // index+thumb not pinching
                else
                {
                    first_pinch = true;
                    angle_adj_stop = false;
                    T = 0.0f;
                }

            }

            // if hand not in view
            else
            {
              //  hand_warn.enabled = true;
            }
        }


        // draw line
        else
        {
            // use middle finger + thumb pinch to allow drawing
            if (Vector3.Magnitude(middle_pos - thumb_pos) < 0.02)
            {
                if (first_pinch)
                {
                    first_pinch = false;
                    CreateLine(index_pos);    // line follows index tip pos
                }

                UpdateLine(index_pos);

            }

            else
            {
                first_pinch = true;
            }

        }


    }






// functions //

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


    void CreateLine(Vector3 indexpos)
    {

        fingerPositions.Clear();
        fingerPositions.Add(indexpos);
        fingerPositions.Add(indexpos);
        line_renderer.positionCount = 2;
        line_renderer.SetPosition(0, fingerPositions[0]);
        line_renderer.SetPosition(1, fingerPositions[1]);
    }

    // continues to draw line
    void UpdateLine(Vector3 indexpos)
    {
        fingerPositions.Add(indexpos);
        line_renderer.positionCount++;
        line_renderer.SetPosition(line_renderer.positionCount - 1, indexpos);
    }

    // clear drawn line
    public void ClearLine()
    {
        line_renderer.positionCount = 0;
    }

    // toggle between teleop and planning mode
    public void ToggleMode()
    {
        teleop = !teleop;
    }



    public void StartLineFollow()
    {
        line_follow_start = true;
        line_follow = true;

        
    }

    public void StopLineFollow()
    {
        line_follow = false;
        EE_virt_pos.GetComponent<Renderer>().enabled = false;
    }











}




