using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Net;
using System.Net.Sockets;
using System.Text;
using System.IO;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Complex;

public class SphereCalib : MonoBehaviour
{

    public GameObject calib_sphere_1;
    public GameObject calib_sphere_2;
    public GameObject calib_sphere_3;
    public GameObject calib_sphere_4;
    public GameObject calib_sphere_5;
    // GameObject[] array;
    //array = new GameObject[5];

   
    int count = 0;
    Vector3[] EE_pos = new Vector3[5];

    bool need_calib = true;

    public GameObject calib_button;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    //array [0] = calib_sphere_1;
    //array [1] = calib_sphere_2;
    //array [2] = calib_sphere_3;
    //array [3] = calib_sphere_4;
    //array [4] = calib_sphere_5;

        string path = Path.Combine(Application.persistentDataPath, "EE_pose_calib.txt");

        if (need_calib & count == 5)
        {
            using (StreamWriter writer = new StreamWriter(path, false))
            {
                for (int i = 0; i < 5; i++)
                {
                    writer.WriteLine("\n" + EE_pos[i][0].ToString("r") + " " + EE_pos[i][1].ToString("r") + " " + EE_pos[i][2].ToString("r"));
            }
            }
            need_calib = false;
        }

        if (!need_calib)
        {
            calib_button.SetActive(false);
            calib_sphere_1.GetComponent<Renderer>().enabled = false;
            calib_sphere_2.GetComponent<Renderer>().enabled = false;
            calib_sphere_3.GetComponent<Renderer>().enabled = false;
            calib_sphere_4.GetComponent<Renderer>().enabled = false;
            calib_sphere_5.GetComponent<Renderer>().enabled = false;

        }



        //// least squares via SVD
        //Matrix<float> A = Matrix<float>.Build.Random(9, 3);
        //Vector<float> b = Vector<float>.Build.Random(9);
        //var A_svd = A.Svd();
        //var S = A_svd.S;
        //var U = A_svd.U;

        //Matrix<float> sigma = Matrix<float>.Build.Dense(9, 3);
        //sigma[0, 0] = 


        //var y = S.Inverse() * U.Transpose() * b;



    }


    // when button press, save EE pos
    public void RecordPos()
    {
        if (count < 5)
        {
            EE_pos[count] = UDPComm.EE_pos;
            count += 1;
            
        }
        else
        {
            count = 0;
        }



        if(count == 0)
        {

            calib_sphere_1.GetComponent<Renderer>().enabled = false;

        }
        else if(count == 1)
            {
            
            calib_sphere_2.GetComponent<Renderer>().enabled = false;
        }

        else if(count == 2)
            {

            calib_sphere_3.GetComponent<Renderer>().enabled = false;
        }

        else if(count == 3)
            {

            calib_sphere_4.GetComponent<Renderer>().enabled = false;
        }

        else 
            {

            calib_sphere_5.GetComponent<Renderer>().enabled = false;
        }
    }

}
