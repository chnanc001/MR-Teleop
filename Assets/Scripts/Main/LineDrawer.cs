using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.IO;

public class LineDrawer : MonoBehaviour
{
    // text display for operating mode (teleop or plan)
    public TextMesh teleop_text;
    public TextMesh plan_text;

    bool first_pinch2 = true;

    // for line drawing
    public GameObject line;
    public GameObject current_line;
    public LineRenderer line_renderer;
    public List<Vector3> fingerPositions;

    public GameObject qr_tracker;

    public static bool line_draw;

    public GameObject start_sphere;
    public GameObject end_sphere;
    public GameObject bounding_box;
    float timer;
    bool flag = false;

    // Start is called before the first frame update =
    void Start()
    {
        // create line object for drawing
        current_line = Instantiate(line, Vector3.zero, Quaternion.identity);
        line_renderer = current_line.GetComponent<LineRenderer>();
    }

    // Update is called once per frame
    void Update()
    {
      

        plan_text.GetComponent<Renderer>().enabled = false;

        if (line_draw)
        {
            string drawn_line = Path.Combine(Application.persistentDataPath, "drawn_line_coordinates.txt");
            // display visual feedback that in planning mode
            plan_text.GetComponent<Renderer>().enabled = true;
            //plan_text.transform.position = qr_tracker.transform.position + new Vector3(0.0f, 0.15f, 0.0f);

            // display start and end points
            start_sphere.SetActive(true);
            end_sphere.SetActive(true);
            //bounding_box.SetActive(true);

            // use middle finger + thumb pinch to allow drawing
            if (Vector3.Magnitude(HandTrack.middle_pos - HandTrack.thumb_pos) < 0.02)
            {
                if (first_pinch2)
                {
                    first_pinch2 = false;
                    CreateLine(HandTrack.index_pos);    // line follows index tip pos

                    using (StreamWriter writer = new StreamWriter(drawn_line, false))
                    {
                        timer += Time.deltaTime;
                    string coord = "\n " + timer + " " + HandTrack.index_pos.ToString("R");
                    writer.WriteLine(coord);
                    }

                    }


                UpdateLine(HandTrack.index_pos);
                using (StreamWriter writer = new StreamWriter(drawn_line, true))
                {
                    timer += Time.deltaTime;
                    string coord = "\n " + timer + " " + HandTrack.index_pos.ToString("R");
                    writer.WriteLine(coord);
                }

            }

            else
            {
              
                first_pinch2 = true;
            }
        }

        // if not in planning mode
        else
        {
            // remove visual feedback
            plan_text.GetComponent<Renderer>().enabled = false;

            // remove start and end points
            start_sphere.SetActive(false);
            end_sphere.SetActive(false);
            //bounding_box.SetActive(false);
        }

    }



    // -- functions -- //
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

}

