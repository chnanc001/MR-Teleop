
// places the buttons wrt hololens position
// pseudo head-locked display

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BtnPlace : MonoBehaviour
{

    // buttons for lines
    public GameObject practice_wire_button;
    public GameObject str_wire_button;
    public GameObject s_wire_button;
    public GameObject ang_wire_button;
    public GameObject remov_line_button;

    // button for record data
    public GameObject record_button;
    public GameObject pause_record_button;

    // position of hololens
    public GameObject holo_pos;

    // button for initiating pinch calibration
    public GameObject pinch_calib_button;
    public TextMesh pinch_calib;    // pinch value

    // button for MTM control
    public GameObject MTM_button;

    public TextMesh read;
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        // place the buttons for displaying or removing lines
        practice_wire_button.transform.position = holo_pos.transform.position + new Vector3(-0.4f, 0.04f, 0.35f);
        str_wire_button.transform.position = holo_pos.transform.position + new Vector3(-0.4f, 0.0f, 0.35f);
        s_wire_button.transform.position = holo_pos.transform.position + new Vector3(-0.4f, -0.04f, 0.35f);
        ang_wire_button.transform.position = holo_pos.transform.position + new Vector3(-0.4f, -0.08f, 0.35f);
        remov_line_button.transform.position = holo_pos.transform.position + new Vector3(-0.4f, -0.12f, 0.35f);



        // place the button for starting pinch calib
        pinch_calib_button.transform.position = holo_pos.transform.position + new Vector3(0.3f, -0.04f, 0.4f);
        pinch_calib.transform.position = holo_pos.transform.position + new Vector3(0.37f, -0.04f, 0.42f);

        // place button for MTM control
        MTM_button.transform.position = holo_pos.transform.position + new Vector3(-0.3f, -0.08f, 0.4f);

        // place the button for recording data
        record_button.transform.position = holo_pos.transform.position + new Vector3(-0.3f, 0.0f, 0.4f);

        // recording text
        read.transform.position = holo_pos.transform.position + new Vector3(-0.25f, 0.0f, 0.42f);

        // place pause recording button
        pause_record_button.transform.position = holo_pos.transform.position + new Vector3(0.0f, -0.5f, 0.2f);

    }
}
