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
   



#if !UNITY_EDITOR
using Windows.Networking.Sockets;
using Windows.Networking.Connectivity;
using Windows.Networking;
#endif


public class MainCntrl_Omni : MonoBehaviour
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


    // for recording data
    bool ReaddVRKmsg = false;
    bool StopReaddVRKmsg = false;
    int readcounter = 0;


    public GameObject self_holo;
    public GameObject qr_tracker;
    public GameObject confirm_button;
    float dist2dVRK;


    bool teleop = false;






    public TextMesh teleop_text;
    public TextMesh plan_text;


    // Start is called before the first frame update
    void Start()
    {


        



        // to keep gaze tracking pointer on
        PointerUtils.SetGazePointerBehavior(PointerBehavior.AlwaysOn);

        // visual feedback for teleop
        // hand_warn.enabled = false;
        teleop_text.GetComponent<Renderer>().enabled = false;
        plan_text.GetComponent<Renderer>().enabled = false;

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
        // dVRK seems to send messages faster than hololens runs update loop (this causes delay in messages read by hololens)
        // therefore read in messages from dVRK more than once per update loop to get newest messages
        read_msg_count = 0;
        string incoming_pose = Path.Combine(Application.persistentDataPath, "dVRKpose_omni.txt");
        string incoming_jaw = Path.Combine(Application.persistentDataPath, "dVRKjaw_omni.txt");
        while (read_msg_count < 4)
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

                // reocord data 
                if (ReaddVRKmsg)
                {
                    using (StreamWriter writer = new StreamWriter(incoming_jaw, true))
                    {
                        string jaw = "\n \n Timestamp: " + System.DateTime.UtcNow.Millisecond + "\n dVRK jaw:" + jaw_angle.ToString("R");
                        writer.WriteLine(jaw);

                    }
                }
            }
            // if pose message
            else
            {
                // extract rot and pos
                EE_quat = QuaternionFromMatrix(parser.GetMatrix4X4(dVRK_msg));
                EE_pos = parser.GetPos(dVRK_msg);

                // record data
                if (ReaddVRKmsg)
                {
                    using (StreamWriter writer = new StreamWriter(incoming_pose, true))
                    {
                        string pose = "\n \nTimestamp: " + System.DateTime.UtcNow.Millisecond + "\n dVRK position:" + EE_pos.ToString("R") + "\n dVRK orientation: " + EE_quat.ToString("R");
                        writer.WriteLine(pose);

                    }
                }
            }
            read_msg_count += 1;
        }



        // draw line
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



    public void ReaddVRK()
    {
        readcounter++;
        if (readcounter > 1)
        {
            ReaddVRKmsg = false;
            Debug.Log("stop Reading dVRK");
            readcounter = 0;
        }
        else
        {

            ReaddVRKmsg = true;
            Debug.Log("Reading dVRK");
            readcounter++;
        }


    }







}




