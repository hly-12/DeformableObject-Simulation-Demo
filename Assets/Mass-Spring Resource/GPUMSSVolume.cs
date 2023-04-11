using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using Assets.Script;
using UnityEngine.Rendering;
using System.Reflection;

public class GPUMSSVolume : MonoBehaviour
{
    public enum MyModel
    {
        IcoSphere_low,
        IcoSphere,
        Torus,
        Bunny_Low_Poly,
        Bunny,
        Cow,
        Armadillo,
        Dragon_refine
    };
    public enum DeformationMethod
    {
        MSM_Interior,
        Volume_Constraint,
    }
    [Header("3D model")]
    public MyModel model;

    [Header("Deformation Method")]
    public DeformationMethod deformationMethod;

    [HideInInspector]
    string modelName;

    [Header("Obj Parameters")]
    public float nodeMass = 1.0f;
    public float dt = 0.005f; // have to devide by 20
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    public float velocityDamping = -0.1f;
    public float thresholdError = 0.000001f;

    [Header("Simulation Parameter")]
    public int speed = 1;

    [Header("Geometric Parameters")]
    public bool useInteriorSpring = false;

    [Header("Spring Parameters")]
    public float kStiffness = 100.0f;
    public float kDamping = 1.0f;
    [Header("Constrinat Parameters")]
    public float dStiffness = 1.0f;
    public float vStiffness = 1.0f;

    [Header("Rendering Paramenter")]
    public ComputeShader computeShaderobj;
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private Material material;
    private ComputeShader computeShader;


    [Header("Debugging Setting")]
    public bool outputVolume = false;
    public string volumeFileName = "";
    public int maxFrame = 400;
    public bool renderVolumeText = false;

    public enum LabelPosition
    {
        Top_Left,
        Top_Center,
        Top_Right,

    }


    [Header("Label Data")]
    public LabelPosition position;
    public string Text;
    public int xOffset;
    public int yOffset;
    public int fontSize;
    public Color textColor = Color.white;
    private Rect rectPos;
    private Color color;



    [HideInInspector]
    List<string[]> tableData = new List<string[]>();

    [HideInInspector]
    private int nodeCount;
    private int springCount;
    private int triCount; // size of triangle
    private int tetCount;

    //main  property
    //list position
    Vector3[] Positions;
    Vector3[] WorldPositions;
    Vector3[] Velocities;
    Vector3[] Forces;
    Vector3[] Normals;
    List<Spring> initSpring = new List<Spring>();
    List<Triangle> faces = new List<Triangle>();
    List<Tetrahedron> elements = new List<Tetrahedron>();

    //for render
    private ComputeBuffer vertsBuff = null;
    private ComputeBuffer triBuffer = null;
    //for compute shader
    private ComputeBuffer positionsBuffer = null;
    private ComputeBuffer velocitiesBuffer = null;
    private ComputeBuffer forcesAsIntBuffer = null;

    private ComputeBuffer facesBuffer = null;
    private ComputeBuffer jacobianAsUIntBuffer = null;
    private ComputeBuffer jacobianBuffer = null;
    private ComputeBuffer surfacevolumeUIntBuffer = null;
    private ComputeBuffer rhsBuffer = null;
    private ComputeBuffer tempRhsBuffer = null;
    private ComputeBuffer systemBuffer = null;
    //private ComputeBuffer lambdaBuffer = null;  // not use currently, might use for debugging

    private ComputeBuffer springsBuffer = null;
    private ComputeBuffer triangleBuffer = null;
    private ComputeBuffer triPtrBuffer = null;

    private int updatePosKernel;
    private int mass_springKernel;
    private int computenormalKernel;
    // for volume preservation
    private int jacobianKernel;
    private int computeVolumeKernel;
    private int calculateLambdaKernel;
    private int calculateForceKernel;




    //

    

    struct vertData
    {
        public Vector3 pos;
        public Vector2 uvs;
        public Vector3 norms;
    };
    int[] triArray;
    vertData[] vDataArray;
    private static GameObject obj;
    float totalVolume;
    int frame = 0;

    //void setDeformationMethod()
    //{

    //}
    void setPositionofLabel()
    {
        int w = Screen.width, h = Screen.height;
        switch (position)
        {
            case LabelPosition.Top_Left:
                {
                    rectPos = new Rect(0 + xOffset, yOffset, w, h * 2 / 100);
                    color = textColor;
                }
                break;
            case LabelPosition.Top_Center:
                {
                    rectPos = new Rect((w / 2) + xOffset, yOffset, w, h * 2 / 100);
                    color = textColor;
                }
                break;
            case LabelPosition.Top_Right:
                {
                    rectPos = new Rect((w) + xOffset, yOffset, w, h * 2 / 100);
                    color = textColor;
                }
                break;
        }
        //fontSize = h * 2 / 40;
    }

    void SelectModelName()
    {
        switch (model)
        {
            case MyModel.IcoSphere_low: modelName = "icosphere_low.1"; break;
            case MyModel.IcoSphere: modelName = "icosphere.1"; break;
            case MyModel.Torus: modelName = "torus.1"; break;
            case MyModel.Bunny_Low_Poly: modelName = "bunny_741"; break;
            case MyModel.Bunny: modelName = "bunny.1"; break;
            case MyModel.Cow: modelName = "cow.1"; break;
            case MyModel.Armadillo: modelName = "Armadillo.1"; break;
            case MyModel.Dragon_refine: modelName = "dragon_refine_01.1"; break;
        }
    }
    private void setupMeshData()
    {

        LoadModel.LoadData(modelName, obj);


        Positions = LoadModel.positions.ToArray();
        faces = LoadModel.faces;
        initSpring = LoadModel.springs;
        //initSpring = LoadModel.springs;
        triArray = LoadModel.triangles.ToArray();
        elements = LoadModel.element;

        nodeCount = Positions.Length;
        springCount = initSpring.Count;
        triCount = faces.Count; //
        tetCount = elements.Count;

        WorldPositions = new Vector3[nodeCount];
        Velocities = new Vector3[nodeCount];
        Forces = new Vector3[nodeCount];
        WorldPositions.Initialize();
        Velocities.Initialize();
        Forces.Initialize();

        vDataArray = new vertData[nodeCount];

        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i] = new vertData();
            vDataArray[i].pos = Positions[i];
            vDataArray[i].norms = Vector3.zero;
            vDataArray[i].uvs = Vector3.zero;
        }

        int triBuffStride = sizeof(int);
        triBuffer = new ComputeBuffer(triArray.Length,
            triBuffStride, ComputeBufferType.Default);

        int vertsBuffstride = 8 * sizeof(float);
        vertsBuff = new ComputeBuffer(vDataArray.Length,
            vertsBuffstride, ComputeBufferType.Default);

        LoadModel.ClearData();
    }
    private void setupShader()
    {
        material.SetBuffer(Shader.PropertyToID("vertsBuff"), vertsBuff);
        material.SetBuffer(Shader.PropertyToID("triBuff"), triBuffer);
    }
    private void setBuffData()
    {

        vertsBuff.SetData(vDataArray);
        triBuffer.SetData(triArray);

        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);
    }
    private void setupComputeBuffer()
    {
        positionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        positionsBuffer.SetData(Positions);

        velocitiesBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        velocitiesBuffer.SetData(Velocities);

        UInt3Struct[] forceUintArray = new UInt3Struct[nodeCount];
        forceUintArray.Initialize();

        forcesAsIntBuffer = new ComputeBuffer(nodeCount, sizeof(uint) * 3);
        forcesAsIntBuffer.SetData(forceUintArray);

        springsBuffer = new ComputeBuffer(springCount,
            sizeof(float) + (sizeof(int) * 2));

        Spring[] springArr = initSpring.ToArray();
        springsBuffer.SetData(springArr);

        List<MTriangle> initTriangle = new List<MTriangle>();  //list of triangle cooresponding to node 
        List<int> initTrianglePtr = new List<int>(); //contain a group of affectd triangle to node
        initTrianglePtr.Add(0);

        for (int i = 0; i < nodeCount; i++)
        {
            foreach (Triangle tri in faces)
            {
                if (tri.vertices[0] == i || tri.vertices[1] == i || tri.vertices[2] == i)
                {
                    MTriangle tmpTri = new MTriangle();
                    tmpTri.v0 = tri.vertices[0];
                    tmpTri.v1 = tri.vertices[1];
                    tmpTri.v2 = tri.vertices[2];
                    initTriangle.Add(tmpTri);
                }
            }
            initTrianglePtr.Add(initTriangle.Count);
        }

        triangleBuffer = new ComputeBuffer(initTriangle.Count, (sizeof(int) * 3));
        triangleBuffer.SetData(initTriangle.ToArray());

        triPtrBuffer = new ComputeBuffer(initTrianglePtr.Count, sizeof(int));
        triPtrBuffer.SetData(initTrianglePtr.ToArray());

        List<MTriangle> initTriangleObj = new List<MTriangle>();
        foreach (Triangle tri in faces)
        {
            MTriangle tmpTri = new MTriangle();
            tmpTri.v0 = tri.vertices[0];
            tmpTri.v1 = tri.vertices[1];
            tmpTri.v2 = tri.vertices[2];
            initTriangleObj.Add(tmpTri);
        }

            //for surface volume preservation
            facesBuffer = new ComputeBuffer(initTriangleObj.Count, sizeof(int) * 3);
        facesBuffer.SetData(initTriangleObj.ToArray()) ;

        UInt3Struct[] uInt3Structs = new UInt3Struct[nodeCount];
        Vector3[] initJacobian = new Vector3[nodeCount];
        uint[] initUint = new uint[1];
        float[] initFloat = new float[1];
        UInt3Struct[] tempRHS = new UInt3Struct[1];
        //uint[] initSVolume = new uint[1];

        jacobianAsUIntBuffer = new ComputeBuffer(nodeCount, sizeof(uint) * 3);
        jacobianAsUIntBuffer.SetData(uInt3Structs);
        jacobianBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        jacobianBuffer.SetData(initJacobian);

        surfacevolumeUIntBuffer = new ComputeBuffer(1, sizeof(uint));
        surfacevolumeUIntBuffer.SetData(initUint);
        rhsBuffer = new ComputeBuffer(1, sizeof(uint));
        rhsBuffer.SetData(initUint);
        tempRhsBuffer = new ComputeBuffer(1, sizeof(uint) * 3);
        tempRhsBuffer.SetData(tempRHS);
        systemBuffer = new ComputeBuffer(1, sizeof(uint));
        systemBuffer.SetData(initUint);
        //not initial data
    }
    private float computeSurfaceVolume()
    {
        float vol = 0.0f;
        for (int i = 0; i < triCount; i++)
        {
            int i1 = triArray[i * 3 + 0];
            int i2 = triArray[i * 3 + 1];
            int i3 = triArray[i * 3 + 2];

            //pos1 = Positions[]
            Vector3 pos1 = Positions[i1];
            Vector3 pos2 = Positions[i2];
            Vector3 pos3 = Positions[i3];

            //float area = 0.5f * length(cross(pos2 - pos1, pos3 - pos1));
            //float3 tmp = area * (pos1 + pos2 + pos3);
            //float3 norm = normalize(cross(pos2 - pos1, pos3 - pos1));

            float area = 0.5f * (Vector3.Cross(pos2 - pos1, pos3 - pos1)).magnitude;
            Vector3 tmp = area * (pos1 + pos2 + pos3);
            Vector3 norm = Vector3.Cross(pos2 - pos1, pos3 - pos1).normalized;

            vol += Vector3.Dot(tmp, norm);

        }
        return vol / 9.0f;
    }
    private void setupComputeShader()
    {
        updatePosKernel = computeShader.FindKernel("updatePosKernel");
        mass_springKernel = computeShader.FindKernel("MSSKernel");
        computenormalKernel = computeShader.FindKernel("computenormalKernel");


        computeVolumeKernel = computeShader.FindKernel("computeVolumeKernel");
        jacobianKernel = computeShader.FindKernel("jacobianKernel");
        calculateLambdaKernel = computeShader.FindKernel("calculateLambdaKernel");
        calculateForceKernel = computeShader.FindKernel("calculateForceKernel");

        computeShader.SetInt("nodeCount", nodeCount);
        computeShader.SetInt("springCount", springCount);
        computeShader.SetInt("triCount", triCount);
        computeShader.SetInt("tetCount", tetCount);

        computeShader.SetFloat("dt", dt);
        float restVolume = computeSurfaceVolume();
        totalVolume = restVolume;
        //print(restVolume);
        computeShader.SetFloat("restVolume", restVolume);
        computeShader.SetFloat("nodeMass", nodeMass);

        computeShader.SetFloat("kS", kStiffness);
        computeShader.SetFloat("kD", kDamping);
        computeShader.SetFloat("thresholdError", thresholdError);

        computeShader.SetBuffer(updatePosKernel, "Positions", positionsBuffer);
        computeShader.SetBuffer(updatePosKernel, "Velocities", velocitiesBuffer);
        computeShader.SetBuffer(updatePosKernel, "ForcesAsInt", forcesAsIntBuffer);
        computeShader.SetBuffer(updatePosKernel, "vertsBuff", vertsBuff); //passing to rendering
        computeShader.SetBuffer(updatePosKernel, "SurfacevolumeUInt", surfacevolumeUIntBuffer);
        computeShader.SetBuffer(updatePosKernel, "JacobianVectorUInt", jacobianAsUIntBuffer);
        computeShader.SetBuffer(updatePosKernel, "Jacobian", jacobianBuffer);


        computeShader.SetBuffer(mass_springKernel, "Positions", positionsBuffer);
        computeShader.SetBuffer(mass_springKernel, "Velocities", velocitiesBuffer);
        computeShader.SetBuffer(mass_springKernel, "ForcesAsInt", forcesAsIntBuffer);
        computeShader.SetBuffer(mass_springKernel, "Springs", springsBuffer);


        computeShader.SetBuffer(computeVolumeKernel, "Positions", positionsBuffer);
        computeShader.SetBuffer(computeVolumeKernel, "Jacobian", jacobianBuffer);
        computeShader.SetBuffer(computeVolumeKernel, "JacobianVectorUInt", jacobianAsUIntBuffer);
        computeShader.SetBuffer(computeVolumeKernel, "SurfacevolumeUInt", surfacevolumeUIntBuffer);
        computeShader.SetBuffer(computeVolumeKernel, "Faces", facesBuffer);
        computeShader.SetBuffer(computeVolumeKernel, "Rhs", rhsBuffer);
        computeShader.SetBuffer(computeVolumeKernel, "temprhs", tempRhsBuffer);
        computeShader.SetBuffer(computeVolumeKernel, "System", systemBuffer);

        computeShader.SetBuffer(jacobianKernel, "Positions", positionsBuffer);
        computeShader.SetBuffer(jacobianKernel, "JacobianVectorUInt", jacobianAsUIntBuffer);
        computeShader.SetBuffer(jacobianKernel, "Jacobian", jacobianBuffer);

        computeShader.SetBuffer(jacobianKernel, "Triangles", triangleBuffer);
        computeShader.SetBuffer(jacobianKernel, "TrianglePtr", triPtrBuffer);

        computeShader.SetBuffer(calculateLambdaKernel, "Velocities", velocitiesBuffer);
        computeShader.SetBuffer(calculateLambdaKernel, "ForcesAsInt", forcesAsIntBuffer);
        computeShader.SetBuffer(calculateLambdaKernel, "JacobianVectorUInt", jacobianAsUIntBuffer);
        computeShader.SetBuffer(calculateLambdaKernel, "Jacobian", jacobianBuffer);
        computeShader.SetBuffer(calculateLambdaKernel, "SurfacevolumeUInt", surfacevolumeUIntBuffer);
        computeShader.SetBuffer(calculateLambdaKernel, "Rhs", rhsBuffer);
        computeShader.SetBuffer(calculateLambdaKernel, "temprhs", tempRhsBuffer);
        computeShader.SetBuffer(calculateLambdaKernel, "System", systemBuffer);


        computeShader.SetBuffer(calculateForceKernel, "ForcesAsInt", forcesAsIntBuffer);
        computeShader.SetBuffer(calculateForceKernel, "JacobianVectorUInt", jacobianAsUIntBuffer);
        computeShader.SetBuffer(calculateForceKernel, "Jacobian", jacobianBuffer);
        computeShader.SetBuffer(calculateForceKernel, "Rhs", rhsBuffer);
        computeShader.SetBuffer(calculateForceKernel, "System", systemBuffer);
        computeShader.SetBuffer(calculateForceKernel, "temprhs", tempRhsBuffer);

        computeShader.SetBuffer(computenormalKernel, "Positions", positionsBuffer);
        computeShader.SetBuffer(computenormalKernel, "Triangles", triangleBuffer);
        computeShader.SetBuffer(computenormalKernel, "TrianglePtr", triPtrBuffer);
        computeShader.SetBuffer(computenormalKernel, "vertsBuff", vertsBuff); //passing to rendering

    }

    void setup()
    {
        material = new Material(renderingShader); // new material for difference object
        material.color = matColor; //set color to material
        computeShader= Instantiate(computeShaderobj); // to instantiate the compute shader to be use with multiple object


        obj = gameObject;
        SelectModelName();
        setupMeshData();
        setupShader();
        setBuffData();
        setupComputeBuffer();
        setupComputeShader();
        setPositionofLabel();
    }

    void Start()
    {
        setup();
        print("nodes :: " + nodeCount);
        print("tris :: " + triCount);
        print("tet :: " + tetCount);
        print("springs :: " + springCount);

        //Application.targetFrameRate = 1000;
    }
    float[] volumeDataGPU = new float[1];
    void dispatchComputeShader()
    {
        for (int i = 0; i < speed; i++)
        {

            switch (deformationMethod)
            {
                case DeformationMethod.MSM_Interior:
                    {
                        computeShader.Dispatch(mass_springKernel,
                         (int)Mathf.Ceil(springCount / 1024.0f), 1, 1);
                        computeShader.Dispatch(computeVolumeKernel,
                           (int)Mathf.Ceil(triCount / 1024.0f), 1, 1);
                    }
                    break;
                case DeformationMethod.Volume_Constraint:
                    {
                        computeShader.Dispatch(mass_springKernel,
                          (int)Mathf.Ceil(springCount / 1024.0f), 1, 1);
                        computeShader.Dispatch(computeVolumeKernel,
                           (int)Mathf.Ceil(triCount / 1024.0f), 1, 1);
                        computeShader.Dispatch(jacobianKernel,
                          (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
                        computeShader.Dispatch(calculateLambdaKernel,
                            (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
                        computeShader.Dispatch(calculateForceKernel,
                            (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
                    }
                    break;


            }
            //computeShader.Dispatch(mass_springKernel,
            //    (int)Mathf.Ceil(springCount / 1024.0f), 1, 1);
            //computeShader.Dispatch(computeVolumeKernel,
            //   (int)Mathf.Ceil(triCount / 1024.0f), 1, 1);
            //computeShader.Dispatch(jacobianKernel,
            //  (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
            //computeShader.Dispatch(calculateLambdaKernel,
            //    (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
            //computeShader.Dispatch(calculateForceKernel,
            //    (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);

            //if ((i == speed-1)&& renderVolumeText == true)
            //{
            //    surfacevolumeUIntBuffer.GetData(volumeDataGPU);
            //    //print(volumeDataGPU[0]);
            //}


            computeShader.Dispatch(updatePosKernel,
                (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
        }
        computeShader.Dispatch(computenormalKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
    }





    //private void OnGUI()
    //{
    //    if (renderVolumeText)
    //    {
    //        int w = Screen.width, h = Screen.height;
    //        GUIStyle style = new GUIStyle();
    //        Rect rect = rectPos;
    //        style.alignment = TextAnchor.UpperLeft;
    //        style.fontSize = fontSize;
    //        style.normal.textColor = color;

    //        //get volume data;

    //        float currVolume = volumeDataGPU[0];
    //        //currVolume = computeSurfaceVolume();

    //        float vLost = (currVolume / totalVolume) * 100.0f;
    //        string text = string.Format("Volume: {0:0.00} %", vLost);
    //        GUI.Label(rect, text, style);
    //    }
    //}
    void buildDataPerRow(int frame_num, float currVolume)
    {
        List<string> rowData = new List<string>();
        rowData.Add(frame.ToString());

        float Vpercentage = (currVolume / totalVolume) * 100.0f;
        rowData.Add(Vpercentage.ToString());
        tableData.Add(rowData.ToArray());
    }
    void writeTableData(string fileName)
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
        string filePath = "E:/Labdata/Unity/VolumeConstraint/free-fall/" + fileName + ".csv";



        StreamWriter outStream = System.IO.File.CreateText(filePath);
        outStream.WriteLine(sb);
        outStream.Close();
        tableData.Clear();
    }


    string path = "D:/Project/Unity/_________FOR_RESEARCH/result30-05-2022/mss-interior/";
    void Update()
    {
        dispatchComputeShader();
        //setData
        //setPositionofLabel();
        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
        material.SetPass(0);
        Graphics.DrawProcedural(material, bounds, MeshTopology.Triangles, triArray.Length,
            1, null, null, ShadowCastingMode.On, true, gameObject.layer);

        //if (outputVolume)
        //{
        //    buildDataPerRow(frame, volumeDataGPU[0]);
        //    if (frame == maxFrame-1)
        //    {
        //        //volume file name refer to prefix of the output file or method used
        //        writeTableData(volumeFileName+"_"+modelName);
        //        print("write Done");
        //        UnityEditor.EditorApplication.isPlaying = false;
        //    }
        //}


        ////writeImageData(frame, 250,path);

        //if (frame > 400)
        //{
        //    UnityEditor.EditorApplication.isPlaying = false;
        //    print("exit");
        //}

        //print(frame);
        frame++;
    }

    void writeImageData(int f, int maxFrame, string imgPath)
    {
        if (f < maxFrame)
            ScreenCapture.CaptureScreenshot(imgPath + "frame" + f.ToString().PadLeft(3, '0') + ".png");
    }

    private void OnDestroy()
    {
        if (this.enabled)
        {
            vertsBuff.Dispose();
            triBuffer.Dispose();
            positionsBuffer.Dispose();
            velocitiesBuffer.Dispose();
            forcesAsIntBuffer.Dispose();
            facesBuffer.Dispose();
            jacobianAsUIntBuffer.Dispose();
            jacobianBuffer.Dispose();
            surfacevolumeUIntBuffer.Dispose();
            rhsBuffer.Dispose();
            tempRhsBuffer.Dispose();
            systemBuffer.Dispose();
            springsBuffer.Dispose();
            triangleBuffer.Dispose();
            triPtrBuffer.Dispose();
        }
    }

}
