using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using Assets.Script;



public class LoadModel : MonoBehaviour
{
    private static GameObject obj;
    
    public static List<Vector3> positions   = new List<Vector3>();
    public static List<Triangle> faces      = new List<Triangle>(); //for compute
    public static List<Tetrahedron> element = new List<Tetrahedron>();
    public static List<Spring> springs      = new List<Spring>();
    public static List<int> triangles       = new List<int>();//for render
    //for transfor to center
    static int nodeCount;
    static int triCount;
    static int tetCount;
    static int springCount;
    static string modelPath = "/TetGen_Model/Modified model/";
    static string fileName = "559sphere.1";
    //static string fileName = "33cube.1";

    //private bool HasSpringFile = false;
    static List<string> rowData = new List<string>(); //a singal row of data springs
    static List<string[]> tableData = new List<string[]>(); // overall data spring


    private static float computeTetraVolume(Vector3 i1, Vector3 i2, Vector3 i3, Vector3 i4)
    {
        float volume = 0.0f;

        volume = 1.0f / 6.0f
            * (i3.x * i2.y * i1.z - i4.x * i2.y * i1.z - i2.x * i3.y * i1.z
            + i4.x * i3.y * i1.z + i2.x * i4.y * i1.z - i3.x * i4.y * i1.z
            - i3.x * i1.y * i2.z + i4.x * i1.y * i2.z + i1.x * i3.y * i2.z
            - i4.x * i3.y * i2.z - i1.x * i4.y * i2.z + i3.x * i4.y * i2.z
            + i2.x * i1.y * i3.z - i4.x * i1.y * i3.z - i1.x * i2.y * i3.z
            + i4.x * i2.y * i3.z + i1.x * i4.y * i3.z - i2.x * i4.y * i3.z
            - i2.x * i1.y * i4.z + i3.x * i1.y * i4.z + i1.x * i2.y * i4.z
            - i3.x * i2.y * i4.z - i1.x * i3.y * i4.z + i2.x * i3.y * i4.z);

        return volume;
    }

    public LoadModel(string filename)
    {
        fileName = filename;
    }
    public LoadModel(string filename,GameObject gameobj)
    {
        fileName = filename;
        obj = gameobj;
    }

    public static void LoadData(string filename, GameObject gameobj)
    {
        fileName = filename;
        obj = gameobj;
        print("start load data !");
        loadNodesPosition();
        loadFaces();
        loadTetrahedron();
        //if (HasSpringFile)
        //    loadSpring();
        //else
        //    writeSpringArr();

        tet2spring();

        //print(nodeCount);
        //print(triCount);
        //print(tetCount);
        //print(springCount);

    }

  

    private static void loadNodesPosition() {
        string Nodepath = Application.dataPath + modelPath + fileName + ".node";
        StreamReader reader1 = new StreamReader(Nodepath);
        string line;
       
        Matrix4x4 sMatrix = Matrix4x4.Scale(obj.transform.localScale);
        Matrix4x4 tMatrix = Matrix4x4.Translate(obj.transform.position);
        Quaternion rotation = Quaternion.Euler(obj.transform.eulerAngles.x,
           obj.transform.eulerAngles.y, obj.transform.eulerAngles.z);
        //Quaternion rotation = Quaternion.identity;
        Matrix4x4 rMatrix = Matrix4x4.Rotate(rotation);

        using (reader1)
        {
            line = reader1.ReadLine();
            do
            {
                line = reader1.ReadLine(); // first line
                if (line != null)
                {
                    string[] tmpPosPerRow = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (tmpPosPerRow.Length > 0 && tmpPosPerRow.Length < 5) // if data
                    {
                        Vector3 pos = new Vector3(0, 0, 0);
                        pos.x = float.Parse(tmpPosPerRow[1]);
                        pos.y = float.Parse(tmpPosPerRow[2]);
                        pos.z = float.Parse(tmpPosPerRow[3]);

                        pos = rMatrix.MultiplyPoint(pos);
                        pos = sMatrix.MultiplyPoint(pos);
                        pos = tMatrix.MultiplyPoint(pos);

                        positions.Add(pos);
                    }
                }
            }
            while (line != null);
            reader1.Close();
        }
        nodeCount = positions.Count;
    }
    private static void loadFaces() {
        string Facepath = Application.dataPath + modelPath + fileName + ".face";
        StreamReader reader1 = new StreamReader(Facepath);
        string line;
        using (reader1)
        {
            line = reader1.ReadLine();
            do
            {
                line = reader1.ReadLine(); // first line
                if (line != null)
                {
                    string[] tmpPosPerRow = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (tmpPosPerRow.Length > 0 && tmpPosPerRow.Length < 6) // if data
                    {
                        //Vector3 face = new Vector3(0, 0, 0);
                        
                        triangles.Add(int.Parse(tmpPosPerRow[1]));
                        triangles.Add(int.Parse(tmpPosPerRow[3]));
                        triangles.Add(int.Parse(tmpPosPerRow[2]));
                        //1,3,2 for torus

                        //faces.Add(Triangle(tmpPosPerRow[1], tmpPosPerRow[3], tmpPosPerRow[2]))
                        Triangle t;
                        if (fileName == "cow.1" || fileName == "dragon_vrip_res3.1")
                            t = new Triangle(int.Parse(tmpPosPerRow[1]), 
                                int.Parse(tmpPosPerRow[2]), int.Parse(tmpPosPerRow[3]));
                        else 
                            t = new Triangle(int.Parse(tmpPosPerRow[1]), 
                                int.Parse(tmpPosPerRow[3]), int.Parse(tmpPosPerRow[2]));

                        faces.Add(t);


                        //face.x = float.Parse(tmpPosPerRow[1]);
                        //face.y = float.Parse(tmpPosPerRow[3]);
                        //face.z = float.Parse(tmpPosPerRow[2]);
                        ////print(face);
                        //faces.Add(face);
                    }
                }
            }
            while (line != null);
            reader1.Close();
        }
        triCount = faces.Count;
    }
    private static void loadTetrahedron() {
        string ElePath = Application.dataPath + modelPath + fileName + ".ele";
        StreamReader reader1 = new StreamReader(ElePath);
        string line;
        using (reader1)
        {
            line = reader1.ReadLine();
            do
            {
                line = reader1.ReadLine(); // first line
                if (line != null)
                {
                    string[] tmpPosPerRow = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (tmpPosPerRow.Length > 0 && tmpPosPerRow.Length < 6) // if data
                    {
                        

                        int i0 = int.Parse(tmpPosPerRow[1]);
                        int i1 = int.Parse(tmpPosPerRow[2]);
                        int i2 = int.Parse(tmpPosPerRow[3]);
                        int i3 = int.Parse(tmpPosPerRow[4]);
                        float volume = computeTetraVolume(positions[i0], positions[i1], 
                            positions[i2], positions[i3]);
                        Tetrahedron tetrahedron = new Tetrahedron(i0, i1, i2, i3, volume);

                        //print(i0 + "," + i1 + "," + i2 + "," + i3);
                        element.Add(tetrahedron);
                    }
                }
            }
            while (line != null);
            reader1.Close();

        }
        tetCount = element.Count;
    }
    private static void loadSpring() {
        string springpath = Application.dataPath + modelPath + fileName + ".node";
        StreamReader reader1 = new StreamReader(springpath);
        string line;

        using (reader1)
        {
            line = reader1.ReadLine();
            do
            {
                line = reader1.ReadLine(); // first line
                if (line != null)
                {
                    string[] tmpPosPerRow = line.Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                    if (tmpPosPerRow.Length > 0 && tmpPosPerRow.Length < 3) // if data
                    {
                        Vector3 pos = new Vector3(0, 0, 0);
                        pos.x = float.Parse(tmpPosPerRow[1]);
                        pos.y = float.Parse(tmpPosPerRow[2]);
                        pos.z = float.Parse(tmpPosPerRow[3]);

                       

                        positions.Add(pos);
                    }
                }
            }
            while (line != null);
            reader1.Close();
        }
        nodeCount = positions.Count;
    }
    private static void writeSpringArr() { 
        for(int i =0; i < springCount; i++)
        {
            buildDataPerRow(i);
        }
        writeTableData();
    }

    static void buildDataPerRow(int i)
    {
        rowData = new List<string>();
        rowData.Add(i.ToString());
        rowData.Add(springs[i].i1.ToString());
        rowData.Add(springs[i].i2.ToString());

        tableData.Add(rowData.ToArray());
    }

    static void writeTableData()
    {
        string[][] output = new string[tableData.Count][];
        for (int i = 0; i < output.Length; i++)
        {
            output[i] = tableData[i];
        }
        int length = output.GetLength(0);
        string delimiter = ",";
        StringBuilder sb = new StringBuilder();

        for (int index = 0; index < length; index++)
            sb.AppendLine(string.Join(delimiter, output[index]));
        string filePath = "Assets/TetGen_Model/" + fileName + ".spring";
       
        StreamWriter outStream = System.IO.File.CreateText(filePath);
        outStream.WriteLine(sb);
        outStream.Close();
        tableData.Clear();
    }

    static Spring initSpring(int i1, int i2)
    {
        float rl = Vector3.Distance(positions[i1], positions[i2]);
        Spring spring = new Spring(i1,i2,rl);
        //print(i1 + "," + i2);

        return spring;
    }
    static void InitSpringList(Spring newSpring)
    {
        bool isDuplicated = false;

        foreach (Spring sp in springs)
        {
            if ((newSpring.i1 == sp.i1 && newSpring.i2 == sp.i2) || (newSpring.i1 == sp.i2 && newSpring.i2 == sp.i1))
            {
                isDuplicated = true;
            }
        }
        if (!isDuplicated)
        {
           springs.Add(newSpring);
           //print(newSpring.i1 + "," + newSpring.i2);
        }



    }

    static void tet2spring()
    {
        //foreach (Tetrahedron t in element)
        //{
        //    InitSpringList(initSpring(t.i1, t.i2));
        //    InitSpringList(initSpring(t.i1, t.i3));
        //    InitSpringList(initSpring(t.i1, t.i4));
        //    InitSpringList(initSpring(t.i2, t.i3));
        //    InitSpringList(initSpring(t.i2, t.i4));
        //    InitSpringList(initSpring(t.i3, t.i4));
        //}

        for(int i = 0; i < tetCount; i++)
        {
            Tetrahedron t = element[i];

            //print(t.i1 + "," + t.i2 + "," + t.i3 + "," + t.i4);

            InitSpringList(initSpring(t.i1, t.i2));
            InitSpringList(initSpring(t.i1, t.i3));
            InitSpringList(initSpring(t.i1, t.i4));
            InitSpringList(initSpring(t.i2, t.i3));
            InitSpringList(initSpring(t.i2, t.i4));
            InitSpringList(initSpring(t.i3, t.i4));
        }

        springCount = springs.Count;
    }

    public static void ClearData()
    {
        positions = new List<Vector3>();
        faces   = new List<Triangle>(); //for compute
        element     = new List<Tetrahedron>();
        springs   = new List<Spring>();
        triangles = new List<int>();//for render

        nodeCount = 0;
        springCount = 0;
        triCount = 0;
        tetCount = 0;
    }
}
