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

public class Calib : MonoBehaviour
{

    // declare variables //

    // for pinch calib
    int pinch_counter = 0;
   public static bool pinch_calibration_complete = false;
    bool pinch_calibration_ongoing = false;
    float calibration_timer = 0;
    float[] calibration_pinch = new float[5];
    public static float calibrated_pinch;
    float counter = 0;
    float timer_hold;
    public TextMesh pinch;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        // pinch calib
        if (pinch_counter > 0 & pinch_counter < 6)
        {
            StopAllCoroutines();
            if (pinch_calibration_ongoing)
            {
                while_pinch();

                StartCoroutine(while_pinch());
                // timer_hold = Time.deltaTime;
              //  Debug.Log(calibration_timer);
            }

            calibrated_pinch = (calibration_pinch[0] + calibration_pinch[1] + calibration_pinch[2] + calibration_pinch[3] + calibration_pinch[4]) / pinch_counter;
            //Debug.Log("calibrated_pinch: " + calibrated_pinch);
        }

        if (pinch_calibration_complete)
        {
            pinch.text = "Done Calibrating";
            
        }
        else
        {
            pinch.text = calibrated_pinch.ToString();

        }

        pinch.GetComponent<Renderer>().enabled = true;
    }




    // --- functions --- //

    public void pinch_calibration()
    {
        pinch_counter++;
        pinch_calibration_ongoing = true;
        pinch_calibration_complete = false;

        Debug.Log("pinch_counter: " + pinch_counter);
        if (pinch_counter > 5)
        {
            pinch_counter = 0;
            pinch_calibration_complete = true;
            pinch_calibration_ongoing = false;

            
            Debug.Log("Done calibrating");
        }
    }

    IEnumerator while_pinch()
    {

       // if (pinch_counter < 6)
        //{
            while (calibration_timer < 5.0f)

            //while(counter<5000)  
            {
                //yield return new WaitForSeconds(2);

                calibration_pinch[pinch_counter - 1] += HandTrack.pinch_dist;
                calibration_timer += Time.deltaTime;
                counter++;
                yield return null;

               
            }
            calibration_timer = 0;
            //Debug.Log(calibration_pinch[pinch_counter - 1]);
            calibration_pinch[pinch_counter - 1] /= counter;
            counter = 0;
            pinch_calibration_ongoing = false;
            //Debug.Log(pinch_calibration_ongoing);
        }

    //}

}

