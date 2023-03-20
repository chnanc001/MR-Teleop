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

using System.Threading.Tasks;

#if !UNITY_EDITOR
using Windows.Networking.Sockets;
using Windows.Networking.Connectivity;
using Windows.Networking;
#endif

#if WINDOWS_UWP
using System.Threading.Tasks;
#endif

public class MainCntrl_test : MonoBehaviour
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
    Vector3 new_EE_pos = Vector3.zero;
    Quaternion new_EE_rot;
    private Vector3 new_EE_pos_send;
    Vector3 EE_start_pos;
    Quaternion EE_start_rot;
    float scale = 0.2f;
    Quaternion thumb_prox_rot;
    Quaternion scaled_new_EE_rot;
    Matrix<float> new_EE_rot_send = Matrix<float>.Build.Random(3, 3);
    Quaternion rot_off;
    
    
    Quaternion index_rot;

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


#if WINDOWS_UWP
    //byte[] data;
    //Socket socket;
    //EndPoint remote;
#endif



    public GameObject UDP;



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



    Matrix<float> EE_positions = Matrix<float>.Build.Random(3, 3);


    public TextMesh teleop_text;
    public TextMesh plan_text;


    bool mode_overwrite = false;

    private Queue<Vector3> EE_pos_q = new Queue<Vector3>();

    [SerializeField]
    private MixedRealityInputAction holdAction = MixedRealityInputAction.None;



    Vector3 prev_filtered_EE_pos = Vector3.zero;


    public float fcmin;
    public float beta;

    public int mov_avg_val;

    public float threshold;


    Vector3 wrist_pos;
    Quaternion wrist_rot;

    Vector3 index_meta_pos;
    Quaternion index_meta_rot;
    Vector3 index_dis_pos;
    Quaternion index_dis_rot;
    Vector3 thumb_dis_pos;
    Quaternion thumb_dis_rot;
    Vector3 start_normal;

    bool rot = false;


    Quaternion prev_EE_rot;


    public GameObject cube;


    Vector3 pos_off;



    Quaternion prev_hand_rot = Quaternion.identity;


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

        //#if WINDOWS_UWP
        //// udp using socket
        //        data = new byte[1024];
        //        ip = new IPEndPoint(IPAddress.Any, 48051);
        //        socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        //        socket.Bind(ip);
        //        sender = new IPEndPoint(IPAddress.Any, 0);
        //        remote = (EndPoint)(sender);
        //#endif






        // create line object for drawing
        current_line = Instantiate(line, Vector3.zero, Quaternion.identity);
        line_renderer = current_line.GetComponent<LineRenderer>();

        // create sphere for EE pos visualisation
        EE_virt_pos = Instantiate(sphere_marker, this.transform);










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



        Debug.Log("qr orientation: " + qr_tracker.transform.rotation);




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
        // get wrist pose
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Right, out pose))
        {
            wrist_pos = pose.Position;
            wrist_rot = pose.Rotation;
        }

        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexMetacarpal, Handedness.Right, out pose))
        {
            index_meta_pos = pose.Position;
            index_meta_rot = pose.Rotation;
        }
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexDistalJoint, Handedness.Right, out pose))
        {
            index_dis_pos = pose.Position;
            index_dis_rot = pose.Rotation;
        }
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbDistalJoint, Handedness.Right, out pose))
        {
            thumb_dis_pos = pose.Position;
            thumb_dis_rot = pose.Rotation;
        }


        Vector3 hand_vect1 = index_dis_pos - index_meta_pos;
        Vector3 hand_vect2 = thumb_dis_pos - index_meta_pos;
        Vector3 hand_normal = Vector3.Cross(hand_vect1, hand_vect2);
        //hand_normal.Normalize();    // normalise normal vector



        //#if WINDOWS_UWP

        //Task task = new Task(
        //async() =>
        //{
        //        // dVRK seems to send messages faster than hololens runs update loop (this causes delay in messages read by hololens)
        //        // therefore read in messages from dVRK more than once per update loop to get newest messages


        //        // read in message from dVRK
        //        data = new byte[1024];
        //        socket.ReceiveFrom(data, ref remote);
        //        dVRK_msg = Encoding.UTF8.GetString(data);
        //        Debug.Log(dVRK_msg);


        //        });

        //        Debug.Log("trying to start task");
        //        task.Start();


        //#endif


        //Debug.Log(UDPRead.dVRK_msg);

        //jaw_angle = UDPRead.jaw_angle;
        //EE_quat = UDPRead.EE_quat;
        //EE_pos = UDPRead.EE_pos;

        //Debug.Log(jaw_angle);
        //Debug.Log(EE_quat);
        //Debug.Log(EE_pos);


        //dVRK_msg = UDPRead.dVRK_msg;

        read_msg_count = 0;
        string incoming_pose = Path.Combine(Application.persistentDataPath, "dVRKpose.txt");
        string incoming_jaw = Path.Combine(Application.persistentDataPath, "dVRKjaw.txt");
        //// read in message from dVRK
        while (read_msg_count < 2)
        { 
            //data = new byte[1024];
            socket.ReceiveFrom(data, ref remote);
            dVRK_msg = Encoding.UTF8.GetString(data);
            // will have to be removed 
            if (dVRK_msg == null)
            {
                new_EE_pos = Vector3.zero;
            }

            // determine if jaw or pose message
            //jaw_match = parser.StringMatch(dVRK_msg, "\"jaw/setpoint_js\":");
            // if jaw message
            if (parser.StringMatch(dVRK_msg, "\"jaw/setpoint_js\":"))
            {
                // extract jaw angle
                jaw_angle = parser.GetJawAngle(dVRK_msg);

                // reocord data 
                if (ReaddVRKmsg)
                {
                    using (StreamWriter writer = new StreamWriter(incoming_jaw, true))
                    {
                        string jaw = "\n \n Timestamp: " + System.DateTime.UtcNow + "\n dVRK jaw:" + jaw_angle.ToString("R");
                        writer.WriteLine(jaw);

                    }
                }


            }
            // if pose message
            else if (parser.StringMatch(dVRK_msg, "\"setpoint_cp\":"))
            {

                // extract rot and pos
                EE_quat = QuaternionFromMatrix(parser.GetMatrix4X4(dVRK_msg));
                Matrix4x4 temp = parser.GetMatrix4X4(dVRK_msg);
                //Debug.Log("dVRK rot: " + temp.rotation);
                EE_pos = parser.GetPos(dVRK_msg);

                

                // record data
                if (ReaddVRKmsg)
                {
                    using (StreamWriter writer = new StreamWriter(incoming_pose, true))
                    {
                        string pose = "\n \nTimestamp: " + System.DateTime.UtcNow + "\n dVRK position:" + EE_pos.ToString("R") + "\n dVRK orientation: " + EE_quat.ToString("R");
                        writer.WriteLine(pose);

                    }
                }
            }

            else
            {
                new_EE_pos = Vector3.zero;
            }

            read_msg_count += 1;
        }

        //// read in message from dVRK
        //data = new byte[1024];
        //socket.ReceiveFrom(data, ref remote);
        //dVRK_msg = Encoding.UTF8.GetString(data);
        //// determine if jaw or pose message
        //jaw_match = parser.StringMatch(dVRK_msg, "\"jaw/setpoint_js\":");
        //// if jaw message
        //if (jaw_match)
        //{
        //    // extract jaw angle
        //    jaw_angle = parser.GetJawAngle(dVRK_msg);

        //    // reocord data 
        //    if (ReaddVRKmsg)
        //    {
        //        using (StreamWriter writer = new StreamWriter(incoming_jaw, true))
        //        {
        //            string jaw = "\n \n Timestamp: " + System.DateTime.UtcNow + "\n dVRK jaw:" + jaw_angle.ToString("R");
        //            writer.WriteLine(jaw);

        //        }
        //    }


        //}
        //// if pose message
        //else
        //{
        //    // extract rot and pos
        //    EE_quat = QuaternionFromMatrix(parser.GetMatrix4X4(dVRK_msg));
        //    Matrix4x4 temp = parser.GetMatrix4X4(dVRK_msg);
        //    //Debug.Log("dVRK rot: " + temp.rotation);
        //    EE_pos = parser.GetPos(dVRK_msg);

        //    // record data
        //    if (ReaddVRKmsg)
        //    {
        //        using (StreamWriter writer = new StreamWriter(incoming_pose, true))
        //        {
        //            string pose = "\n \nTimestamp: " + System.DateTime.UtcNow + "\n dVRK position:" + EE_pos.ToString("R") + "\n dVRK orientation: " + EE_quat.ToString("R");
        //            writer.WriteLine(pose);

        //        }
        //    }
        //}









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

            }

            EE_virt_pos.transform.position = EE_virt_pos_start + Vector3.Scale((EE_line_start_pos - EE_pos), new Vector3(1.0f, -1.0f, -1.0f));

        }



        // control mode via distance of self from dVRK
        if (confirm_button.activeSelf == false)    // i.e. the position of QR code (and dVRK) is confirmed
        {
            dist2dVRK = Vector3.Magnitude(self_holo.transform.position - qr_tracker.transform.position);

            if (dist2dVRK < 1.0f & !mode_overwrite)
            {
                teleop = false;    // can only enter plan mode

                if (HandPoseUtils.PinkyFingerCurl(Handedness.Right) < 0.1)
                {
                    mode_overwrite = true;
                }

            }

            else if (dist2dVRK > 1.0f | mode_overwrite)
            {
                teleop = true;    // can teleop

                Debug.Log(HandPoseUtils.PinkyFingerCurl(Handedness.Right));
                if (HandPoseUtils.IndexFingerCurl(Handedness.Right) < 0.1)
                {
                    mode_overwrite = false;
                }

            }

        }




        // can only plan within certain distance from dVRK












        teleop = true;
        if (teleop)
        {


            plan_text.GetComponent<Renderer>().enabled = false;
            teleop_text.GetComponent<Renderer>().enabled = true;
            teleop_text.transform.position = qr_tracker.transform.position + new Vector3(0.0f, 0.15f, 0.0f);




            // teleop //
            // if hand is in view
            if (thumb_prox.GetComponent<Renderer>().enabled)
            {

                //  hand_warn.enabled = false;

                // if index+thumb == pinching -> can teleoperate
                if (Vector3.Magnitude(index_pos - thumb_pos) < 0.018)
                {

                    if (first_pinch)
                    {
                        first_pinch = false;

                        Debug.Log("first pinch");

                        // get starting hand position
                        hand_start_pos = self_holo.transform.position + index_pos;
                        hand_start_rot = self_holo.transform.rotation * index_rot;
                        //hand_start_rot = index_rot;

                        // save start position of EE
                        if (new_EE_pos == Vector3.zero | EE_pos == new_EE_pos)
                        {
                            Debug.Log("same");
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

                        

                        // starting orientation between wrist and EE
                        //rot_off = Quaternion.Inverse(hand_start_rot) * EE_start_rot;

                        start_normal = hand_normal;

                        // get rotation diff between wrist and torus
                        //rot_off = Quaternion.Inverse(index_rot) * cube.transform.rotation;
                        //pos_off = index_pos - cube.transform.position;
                        //Quaternion index_rot_edit = new Quaternion(-index_rot.x, -index_rot.z, index_rot.y, index_rot.w);
                        rot_off = Quaternion.Inverse(self_holo.transform.rotation * index_rot) * EE_start_rot;
                        pos_off = index_pos - EE_pos;
                        prev_EE_rot = self_holo.transform.rotation * index_rot;



                    }


                    // write pose message //




                    //if (Vector3.Magnitude(index_pos - prev_index_pos))
                    //{

                    //}





                    Vector3 hand_trans = Vector3.Scale(((self_holo.transform.position + index_pos) - hand_start_pos), new Vector3(-1.0f, 1.0f, -1.0f));

                    //if (EE_pos_q.Count < mov_avg_val)    // just makes it lagging, still jerky
                    //{
                    //    EE_pos_q.Enqueue(hand_trans);
                    //}
                    //else
                    //{
                    //    EE_pos_q.Dequeue();
                    //    EE_pos_q.Enqueue(hand_trans);
                    //}

                    //hand_trans = MovingAveragePos(EE_pos_q);

                    // translation
                    // get new EE position as scaled of how much hand has moved, relative translation
                    //new_EE_pos = EE_start_pos + scale * Vector3.Scale(((self_holo.transform.position + index_pos) - hand_start_pos), new Vector3(-1.0f, 1.0f, -1.0f));

                    //hand_trans = new Vector3(hand_trans.x, hand_trans.z, hand_trans.y);
                    //hand_trans = EE_start_rot * hand_trans;

                    //EE_start_pos = new Vector3(EE_start_pos.x, EE_start_pos.z, EE_start_pos.y);

                    new_EE_pos = EE_start_pos + scale * hand_trans;
                    //new_EE_pos_send = EE_start_pos + scale * hand_trans;

                    //new_EE_pos = EE_start_pos;

                    // convert pos to same orientation as dVRK i.e. y and z swapped
                    new_EE_pos_send[0] = new_EE_pos[0];
                    new_EE_pos_send[1] = new_EE_pos[2];
                    new_EE_pos_send[2] = new_EE_pos[1];

                    //if (prev_filtered_EE_pos == Vector3.zero)
                    //{
                    //    prev_filtered_EE_pos = new_EE_pos_send;
                    //}


                    //float alpha = SmoothingCoeff(fcmin, beta, new_EE_pos_send, prev_filtered_EE_pos);

                    //new_EE_pos_send = OneEuroFilter(alpha, new_EE_pos_send, prev_filtered_EE_pos);
                    //prev_filtered_EE_pos = new_EE_pos_send;







                    // ROTATION //

                    // get new rot of EE based off hand rot, relative rotation (not absolute)
                    //new_EE_rot = EE_start_rot * (Quaternion.Inverse(hand_start_rot) * self_holo.transform.rotation * thumb_prox_rot);
                    //new_EE_rot = self_holo.transform.rotation * thumb_prox_rot * rot_off;



                    //Debug.Log("euler angles: " + self_holo.transform.eulerAngles);


                    //Quaternion index_rot_edit2 = new Quaternion(-index_rot.x, -index_rot.z, index_rot.y, index_rot.w);
                    //Quaternion holo_rot = self_holo.transform.rotation;
                    //Quaternion holo_rot_edit = new Quaternion(-holo_rot.x, -holo_rot.z, holo_rot.y, holo_rot.w);


                    //Quaternion hand_rot = (Quaternion.Inverse(prev_EE_rot) * self_holo.transform.rotation * index_rot);

                    Quaternion holo_2_endo = new Quaternion(0, 1, 0, 0);

                    //hand_start_rot = holo_2_endo * hand_start_rot;
                    //index_rot = holo_2_endo * index_rot;

                    //Quaternion hand_rot = (Quaternion.Inverse(hand_start_rot) * self_holo.transform.rotation * index_rot);

                    Quaternion hand_rot = (Quaternion.Inverse(Quaternion.Inverse(holo_2_endo) * hand_start_rot) * Quaternion.Inverse(holo_2_endo) * self_holo.transform.rotation * index_rot );


                    //hand_rot = hand_rot * holo_2_endo; 


                    Vector3 axis;
                    float angle;
                    hand_rot.ToAngleAxis(out angle, out axis);






                    //Vector3 hand_rot_angles = hand_rot.eulerAngles;


                    ////Vector3 prev_hand_rot_angles = prev_hand_rot.eulerAngles;

                    //Vector3 angles = hand_rot_angles; // - prev_hand_rot_angles;
                    //Vector3 angles2 = new Vector3(Math.Abs(angles.x), Math.Abs(angles.y), Math.Abs(angles.z));

                    //Quaternion send_quat;

                    //if (angles2.x > angles2.y)
                    //{
                    //    if (angles2.x > angles.z)
                    //    {
                    //        // send x angle
                    //        send_quat = Quaternion.AngleAxis(angles.x, Vector3.right);
                    //    }
                    //    else
                    //    {
                    //        // send z angle
                    //        send_quat = Quaternion.AngleAxis(angles.z, Vector3.forward);
                    //    }
                    //}
                    //else if (angles.y > angles.z)
                    //{
                    //    // send y angle
                    //    send_quat = Quaternion.AngleAxis(angles.y, Vector3.up);
                    //}
                    //else
                    //{
                    //    // send z angle
                    //    send_quat = Quaternion.AngleAxis(angles.z, Vector3.forward);
                    //}

                    //new_EE_rot = send_quat * EE_start_rot;

                    //prev_hand_rot = hand_rot;


                    //Debug.Log("send quat: " + send_quat);



                    Vector3 send_axis;
                    float send_angle;

                    if (axis.x > axis.y)
                    {
                        if (axis.x > axis.z)
                        {
                            // send rot in x axis
                            send_axis = Vector3.right;
                            send_angle = hand_rot.eulerAngles.x;

                        }
                        else
                        {
                            // send in z axis
                            send_axis = Vector3.forward;
                            send_angle = hand_rot.eulerAngles.z;
                        }
                    }
                    else if (axis.y > axis.z)
                    {
                        // send rot in y axis
                        send_axis = Vector3.up;
                        send_angle = hand_rot.eulerAngles.y;
                    }
                    else
                    {
                        // send rot in z axis
                        send_axis = Vector3.forward;
                        send_angle = hand_rot.eulerAngles.z;
                    }


                    //hand_rot = Quaternion.AngleAxis(send_angle, send_axis);
                    //Debug.Log("adjusted hand rot: " + hand_rot);

                    new_EE_rot = hand_rot * EE_start_rot;

                    //Quaternion hand_rot = (Quaternion.Inverse(hand_start_rot) * index_rot);

                    //hand_rot = new Quaternion(-hand_rot.x, -hand_rot.z, hand_rot.y, hand_rot.w);

                    //hand_rot = new Quaternion(0.0f, 1.0f, 0.0f, 0.0f) * hand_rot;


                    //Quaternion hand_rot = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);

                    //if (rot)
                    //{
                    //    hand_rot = Quaternion.AngleAxis(10, Vector3.right);
                    //    //rot = false;
                    //}



                    //new_EE_rot = self_holo.transform.rotation * index_rot * Quaternion.Inverse(hand_rot) * rot_off * hand_rot;
                    //hand_rot = new Quaternion(-hand_rot.x, hand_rot.y, hand_rot.z, hand_rot.w);
                    //hand_rot = hand_rot * Quaternion.AngleAxis(-180, Vector3.right);


                    prev_EE_rot = self_holo.transform.rotation * index_rot;


                    //Debug.Log("rot start: " + start_normal);
                    //Debug.Log("hand normal: " + hand_normal);

                    //Debug.Log("angle of rot: " + Vector3.Dot(start_normal, hand_normal) / (Vector3.Magnitude(start_normal) * Vector3.Magnitude(hand_normal)));


                    //float angle_x = Mathf.Acos(Vector3.Dot(new Vector3(0.0f, start_normal[1], start_normal[2]), new Vector3(0.0f, hand_normal[1], hand_normal[2])));
                    //float angle_y = Mathf.Acos(Vector3.Dot(new Vector3(start_normal[0], 0.0f, start_normal[2]), new Vector3(hand_normal[0], 0.0f, hand_normal[2])));
                    //float angle_z = Mathf.Acos(Vector3.Dot(new Vector3(start_normal[0], start_normal[1], 0.0f), new Vector3(hand_normal[0], hand_normal[1], 0.0f)));

                    //Debug.Log("x_angle: " + Mathf.Rad2Deg * angle_x);
                    //Debug.Log("y_angle: " + Mathf.Rad2Deg * angle_y);
                    //Debug.Log("z_angle: " + Mathf.Rad2Deg * angle_z);


                    //Quaternion hand_rot = Quaternion.Euler(Mathf.Rad2Deg * (float)angle_z, Mathf.Rad2Deg * (float)angle_y, Mathf.Rad2Deg * (float)angle_x);
                    //Debug.Log("hand rot: " + hand_rot);
                    //new_EE_rot = EE_start_rot * hand_rot;
                    //Debug.Log("new EE rot: " + new_EE_rot);



                    // using PLANE method
                    //float rot_angle = Mathf.Acos(Vector3.Dot(start_normal, hand_normal) / (Vector3.Magnitude(start_normal) * Vector3.Magnitude(hand_normal)));
                    //Vector3 rot_axis = Vector3.Cross(start_normal, hand_normal);
                    //rot_axis = rot_axis / Vector3.Magnitude(rot_axis);

                    //// axis angle to quaternion
                    ////Quaternion hand_rot = new Quaternion(rot_axis[0] * Mathf.Sin(rot_angle / 2), rot_axis[1] * Mathf.Sin(rot_angle / 2), rot_axis[2] * Mathf.Sin(rot_angle / 2), Mathf.Cos(rot_angle / 2));
                    //Quaternion hand_rot = Quaternion.AngleAxis(Mathf.Rad2Deg * rot_angle, rot_axis);
                    //hand_rot = new Quaternion(hand_rot.z, hand_rot.x, hand_rot.y, hand_rot.w);



                    //Debug.Log("EE euler angles" + EE_start_rot);
                    //Debug.Log("rot trans: " + hand_rot);

                    //Quaternion hand_rot = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);

                    //if (rot)
                    //{
                    //    hand_rot = Quaternion.AngleAxis(10, Vector3.forward);
                    //    //rot = false;
                    //}


                    //new_EE_rot = hand_rot * EE_start_rot;




                    //new_EE_rot = Quaternion.Inverse(prev_EE_rot) * hand_rot * prev_EE_rot;





                    // scale quaternion
                    scaled_new_EE_rot = Quaternion.Slerp(EE_start_rot, new_EE_rot, 1f);

                    //Quaternion res_rot = new Quaternion(0.0f, 0.0f, scaled_new_EE_rot[2], scaled_new_EE_rot[3]);


                    // convert normalised quaternion to rotation matrix
                    new_EE_rot_send = Quat2Rot(Quaternion.Normalize(scaled_new_EE_rot));
                    //new_EE_rot_send = Quat2Rot(Quaternion.Normalize(res_rot));

                    //EE_positions.Column(0);


                    






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



                    //UDP.GetComponent<UDPRead>().UDPsend(pose_message, jaw_message);


                    //// send json strings to dVRK //
                    send_msg = Encoding.UTF8.GetBytes(pose_message);
                    socket.SendTo(send_msg, remote);
                    send_msg = Encoding.UTF8.GetBytes(jaw_message);
                    socket.SendTo(send_msg, remote);



                    //read_msg_count = 0;
                    //while (read_msg_count < 10)
                    //{
                    //    // read in message from dVRK
                    //    data = new byte[1024];
                    //    socket.ReceiveFrom(data, ref remote);
                    //    dVRK_msg = Encoding.UTF8.GetString(data);

                    //    read_msg_count += 1;
                    //}

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

            plan_text.GetComponent<Renderer>().enabled = true;
            plan_text.transform.position = qr_tracker.transform.position + new Vector3(0.0f, 0.15f, 0.0f);
            teleop_text.GetComponent<Renderer>().enabled = false;

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

    // moving average
    private Vector3 MovingAveragePos(Queue<Vector3> EE_pos_q)
    {
        Vector3 sum = new Vector3(0.0f, 0.0f, 0.0f);
      

        for (int i = 0; i < EE_pos_q.Count; i++)
        {
            sum += EE_pos_q.ElementAt(i);
        }

        return sum / EE_pos_q.Count;

    }


    private float SmoothingCoeff(float fcmin, float beta, Vector3 new_EE_pos_send, Vector3 prev_filtered_EE_pos)
    {
        float fc = fcmin + beta * (Vector3.Magnitude(new_EE_pos_send - prev_filtered_EE_pos) / Time.deltaTime);
        float tau = 1 / (2 * (float)Math.PI * fc);
        float alpha = 1 / (1 + (tau / Time.deltaTime));
        return alpha;
    }


    private Vector3 OneEuroFilter(float alpha, Vector3 new_EE_pos_send, Vector3 prev_filtered_EE_pos)
    {
        return alpha * new_EE_pos_send + (1 - alpha) * prev_filtered_EE_pos;
    }

    private async Task ReadUDP()
    {
        // read in message from dVRK
        data = new byte[1024];
        socket.ReceiveFrom(data, ref remote);
        dVRK_msg = Encoding.UTF8.GetString(data);
        Debug.Log(dVRK_msg);
    }


    public void Rot()
    {
        Debug.Log("rotate");
        rot = !rot;
    }




}




