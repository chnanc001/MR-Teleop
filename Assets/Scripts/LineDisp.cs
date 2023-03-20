
// script to display and place straight, s- and angled wires when associating button is pressed

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.Input;

using System;

public class LineDisp : MonoBehaviour
{

    // lines
    public GameObject practice_wire;
    public GameObject straight_wire;
    public GameObject s_wire;
    public GameObject angled_wire;

    public GameObject hololens;

    // line drawer to clear drawn line as well
    public GameObject line_drawer;

    // Start is called before the first frame update
    void Start()
    {

        practice_wire.SetActive(false);
        straight_wire.SetActive(false);
        s_wire.SetActive(false);
        angled_wire.SetActive(false);
    }

    // Update is called once per frame
    void Update()
    {

    }


    // --- functions --- //

    public void DispPracticeWire()
    {
        line_drawer.GetComponent<LineDrawer>().ClearLine();
        practice_wire.SetActive(true);
        straight_wire.SetActive(false);
        s_wire.SetActive(false);
        angled_wire.SetActive(false);
        //straight_wire.transform.position = qrPos + new Vector3(0.07f, -0.17f, -1.4f);
        //straight_wire.transform.position = new Vector3(line_offset_x, line_offset_y, line_offset_z);
    }


    public void DispStrWire()
    {
        line_drawer.GetComponent<LineDrawer>().ClearLine();
        ;

        practice_wire.SetActive(false);
        straight_wire.SetActive(true);
        s_wire.SetActive(false);
        angled_wire.SetActive(false);
        //straight_wire.transform.position = qrPos + new Vector3(0.07f, -0.17f, -1.4f);
        //straight_wire.transform.position = new Vector3(line_offset_x, line_offset_y, line_offset_z);
    }

    public void DispSWire()
    {
        line_drawer.GetComponent<LineDrawer>().ClearLine();

        practice_wire.SetActive(false);
        straight_wire.SetActive(false);
        s_wire.SetActive(true);
        angled_wire.SetActive(false);
        //s_wire.transform.position = qrPos + new Vector3(-0.028f, -0.2f, -1.4f);
        //s_wire.transform.position = qrPos + new Vector3(line_offset_x, line_offset_y, line_offset_z);
    }

    public void DispAngledWire()
    {
        line_drawer.GetComponent<LineDrawer>().ClearLine();

        practice_wire.SetActive(false);
        straight_wire.SetActive(false);
        s_wire.SetActive(false);
        angled_wire.SetActive(true);
        //angled_wire.transform.position = qrPos + new Vector3(-0.028f, -0.23f, -1.5f);
        //angled_wire.transform.position = new Vector3(line_offset_x, line_offset_y, line_offset_z);
    }

    public void RemoveLines()
    {
        practice_wire.SetActive(false);
        straight_wire.SetActive(false);
        s_wire.SetActive(false);
        angled_wire.SetActive(false);
        // clear any drawn lines as well
        line_drawer.GetComponent<LineDrawer>().ClearLine();
    }


}
