using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using Assets.Script;
using UnityEngine.Rendering;

public class CPUMSSVolume : MonoBehaviour
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
        Dragon_refine,
        Asian_dragon
    };

    [Header("3D model")]
    public MyModel model;
    [HideInInspector]
    string modelName;
    [Header("Obj Parameters")]
    public float nodeMass = 1.0f;
    public float dt = 0.005f; // have to devide by 20
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    public float velocityDamping = -0.1f;
    public int speed = 1;

    [Header("Spring Parameters")]
    public float kStiffness = 100.0f;
    public float kDamping = 1.0f;
   
   

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
    ComputeBuffer vertsBuff = null;
    ComputeBuffer triBuffer = null;
    [Header("Rendering Paramenter")]
    public Shader renderingShader;
    public Color matColor;
    [HideInInspector]
    private Material material;

    struct vertData
    {
        public Vector3 pos;
        public Vector2 uvs;
        public Vector3 norms;
    };
    int[] triArray;
    vertData[] vDataArray;
    private static GameObject obj;

    //LoadModel objModel = new LoadModel("559sphere.1", obj);

    // Start is called before the first frame update

    float totalVolume;

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

        for(int i = 0;i < nodeCount; i++)
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

         material.SetMatrix("_LocalToWorld", transform.localToWorldMatrix);
         material.SetMatrix("_WorldToLocal", transform.worldToLocalMatrix);
    }

    void setup()
    {
        obj = gameObject;

        SelectModelName();

        setupMeshData();
        setupShader();
        setBuffData();
    }

    void Start()
    {
        material = new Material(renderingShader); // new material for difference object
        material.color = matColor; //set color to material
        setup();
        foreach (Tetrahedron t in elements)
        {
            totalVolume += t.RestVolume;
        }
    }

    void computeSpringForce()
    {
        for (int i = 0; i < springCount; i++)
        {
            int i1 =initSpring[i].i1;
            int i2 =initSpring[i].i2;
            float rL = initSpring[i].RestLength;

            Vector3 p1 = Positions[i1];
            Vector3 p2 = Positions[i2];
            Vector3 v1 = Velocities[i1];
            Vector3 v2 = Velocities[i2];

            Vector3 forceDirection = p2 - p1;
            Vector3 velocityDirection = v2 - v2;

            float leng = Vector3.Distance(p2, p1);
            float springForce = ((leng - rL) * kStiffness);
            float damp = (Vector3.Dot(velocityDirection, forceDirection) / leng) * kDamping;
            Vector3 SpringForce = (springForce + damp) * forceDirection / leng;

            Forces[i1] += SpringForce;
            Forces[i2] -= SpringForce;

        }
    }

    float Rounding(float x, int digit)
    {
        return (Mathf.Floor((x) * Mathf.Pow(10.0f, digit) + 0.5f) / Mathf.Pow(10.0f, digit));
    }
    void computeDistanceConstraintForce()
    {
        for (int i = 0; i < springCount; i++)
        {
            Spring s = initSpring[i];
            int i1 = s.i1;
            int i2 = s.i2;
            float rL = s.RestLength;

            Vector3 p1 = Positions[i1];
            Vector3 p2 = Positions[i2];
            Vector3 v1 = Velocities[i1];
            Vector3 v2 = Velocities[i2];

            //float phi = Mathf.Pow(Vector3.Distance(p2,p1), 2) - Mathf.Pow(rL, 2);
            float phi = Mathf.Pow((p2.x - p1.x), 2) + Mathf.Pow((p2.y - p1.y), 2) +
                        Mathf.Pow((p2.z - p1.z), 2) - Mathf.Pow(rL, 2);
            phi = Rounding(phi, 6);

            float[] phiq = new float[6];
            float[] Vect = new float[6];
            float rhs = 0;
            float sys = 0;

            phiq[0] = 2 * (p1.x - p2.x);
            phiq[1] = 2 * (p1.y - p2.y);
            phiq[2] = 2 * (p1.z - p2.z);

            phiq[3] = 2 * (p2.x - p1.x);
            phiq[4] = 2 * (p2.y - p1.y);
            phiq[5] = 2 * (p2.z - p1.z);

            Vector3 n1 = v1 / dt + gravity + (Forces[i1]);
            Vector3 n2 = v2 / dt + gravity + (Forces[i2]);

            Vect[0] = n1.x;
            Vect[1] = n1.y;
            Vect[2] = n1.z;

            Vect[3] = n2.x;
            Vect[4] = n2.y;
            Vect[5] = n2.z;

            for (int j = 0; j < 6; j++)
            {
                rhs += (phiq[j] * Vect[j]);
                sys += Mathf.Pow(phiq[j], 2);
            }

            rhs += (phi / Mathf.Pow(dt, 2));
            float lambda =  rhs / sys;

            //print(i + ":: " + lambda);
            Vector3 f1 = new Vector3(phiq[0], phiq[1], phiq[2]);
            Vector3 f2 = new Vector3(phiq[3], phiq[4], phiq[5]);

            f1 = f1 * lambda;
            f2 = f2 * lambda;
            Forces[i1] += -f1;
            Forces[i2] += -f2;

        }
    }
    private float calculateTriArea(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        float area = 0.0f;
        float term1, term2, term3;

        term1 = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
        term2 = (p2.x - p1.x) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.x - p1.x);
        term3 = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

        area = 0.5f * Mathf.Sqrt(term1 * term1 + term2 * term2 + term3 * term3);
        return area;
    }
    private float computeSurfaceVolume()
    {
        Vector3[] coefficientVector = new Vector3[nodeCount];
        coefficientVector.Initialize();
        float area = 0.0f;
        Vector3 pos1, pos2, pos3;
        float currentVolume = 0.0f;

        for (int i = 0; i < triCount; i++)
        {
            int i1 = triArray[i * 3 + 0];
            int i2 = triArray[i * 3 + 1];
            int i3 = triArray[i * 3 + 2];

            //pos1 = Positions[]
            pos1 = Positions[i1];
            pos2 = Positions[i2];
            pos3 = Positions[i3];

            area = calculateTriArea(pos1, pos2, pos3);

            Vector3 norm = Vector3.Cross(pos2 - pos1, pos3 - pos1);
            norm = norm.normalized;

            coefficientVector[i1] += (norm * area / 3.0f);
            coefficientVector[i2] += (norm * area / 3.0f);
            coefficientVector[i3] += (norm * area / 3.0f);
        }

        for (int i = 0; i < nodeCount; i++)
        {
            //currentVolume += (coefficientVector[i].x * Positions[i].x)
            //    + (coefficientVector[i].y * Positions[i].y) 
            //    + (coefficientVector[i].z * Positions[i].z);

            currentVolume += (Vector3.Dot(coefficientVector[i], Positions[i]));
        }

        return currentVolume / 3.0f;
    }
    void globalVolumePreservation()
    {
        float system = 0.0f;
        float rhs = 0.0f;
        float lambda = 0.0f;

        float[] uStar = new float[nodeCount];
        float[] vStar = new float[nodeCount];
        float[] wStar = new float[nodeCount];
        Vector3[] jacobianVector = new Vector3[nodeCount];
        uStar.Initialize();
        vStar.Initialize();
        wStar.Initialize();
        jacobianVector.Initialize();

        // Calculate temporary velocity using current status
        for (int i = 0; i < nodeCount; i++)
        {
            uStar[i] = (Velocities[i].x / dt) + gravity.x + Forces[i].x;
            vStar[i] = (Velocities[i].y / dt) + gravity.y + Forces[i].y;
            wStar[i] = (Velocities[i].z / dt) + gravity.z + Forces[i].z;
        }
        // Calculate Jacobian vector 
        for (int i = 0; i < triCount; i++)
        {
            int i1 = triArray[i * 3 + 0];
            int i2 = triArray[i * 3 + 1];
            int i3 = triArray[i * 3 + 2];

            //pos1 = Positions[]
            Vector3 pos1 = Positions[i1];
            Vector3 pos2 = Positions[i2];
            Vector3 pos3 = Positions[i3];

            jacobianVector[i1].x += 0.5f * (pos3.y * pos2.z - pos2.y * pos3.z);
            jacobianVector[i1].y += 0.5f * (-pos3.x * pos2.z + pos2.x * pos3.z);
            jacobianVector[i1].z += 0.5f * (pos3.x * pos2.y - pos2.x * pos3.y);
            jacobianVector[i2].x += 0.5f * (-pos3.y * pos1.z + pos1.y * pos3.z);
            jacobianVector[i2].y += 0.5f * (pos3.x * pos1.z - pos1.x * pos3.z);
            jacobianVector[i2].z += 0.5f * (-pos3.x * pos1.y + pos1.x * pos3.y);
            jacobianVector[i3].x += 0.5f * (pos2.y * pos1.z - pos1.y * pos2.z);
            jacobianVector[i3].y += 0.5f * (-pos2.x * pos1.z + pos1.x * pos2.z);
            jacobianVector[i3].z += 0.5f * (pos2.x * pos1.y - pos1.x * pos2.y);
        }
        // Build system to solve for Lagrange Multipliers	
        // Create system: sys = phiq * phiq'
        system = 0.0f;
        for (int i = 0; i < nodeCount; i++)
        {
            system += (jacobianVector[i].x * jacobianVector[i].x)
                + (jacobianVector[i].y * jacobianVector[i].y)
                + (jacobianVector[i].z * jacobianVector[i].z);
        }

        // Calculate current error 
        float phi = (totalVolume - computeSurfaceVolume());


        //print("phi :: " +phi);
        rhs = phi / (dt * dt);
        float tempX = 0.0f;
        float tempY = 0.0f;
        float tempZ = 0.0f;
        //Vector3 tmpRhs = Vector3.zero;
        for (int i = 0; i < nodeCount; i++)
        {
            tempX += jacobianVector[i].x * uStar[i];
            tempY += jacobianVector[i].y * vStar[i];
            tempZ += jacobianVector[i].z * wStar[i];
            //Vector3 tmpVel = new Vector3(uStar[i], vStar[i], wStar[i]);
            //tmpRhs = Vector3.Cross(jacobianVector[i], tmpVel);

        }
        //print(tempX + tempY + tempZ);
        rhs = rhs + tempX + tempY + tempZ;
        //print(rhs);
        //rhs = rhs + tmpRhs.x + tmpRhs.x + tmpRhs.x;
        lambda = rhs / system;


        for (int i = 0; i < nodeCount; i++)
        {
            Forces[i] += -jacobianVector[i] * lambda;
        }

    }

    void UpdateNodes()
    {
        //compute node
        for (int i = 0; i < nodeCount; i++)
        {
            Vector3 pos = Positions[i];
            Vector3 vel = Velocities[i];

            Vector3 force = gravity + (Forces[i]);

            vel = vel + force * dt;
            pos = pos + vel * dt;

            Positions[i] = pos;
            Velocities[i] = vel;
            Forces[i] = Vector3.zero;
            
            if (Positions[i].y < -2.0f)
            {

                Positions[i].y = -2.0f;
                Velocities[i] *= -0.1f;
            }
            vDataArray[i].pos = Positions[i];
        }
       
    }


    void computeVertexNormal()
    {
        //

        for (int i = 0; i < triCount; i++)
        {
            //print(TriIndices[i * 3 + 0]+","+ TriIndices[i * 3 + 1] + "," +TriIndices[i * 3 + 2]);
            //
            Vector3 v1 = Positions[triArray[i * 3 + 0]];
            Vector3 v2 = Positions[triArray[i * 3 + 1]];
            Vector3 v3 = Positions[triArray[i * 3 + 2]];

            Vector3 N = (Vector3.Cross(v2 - v1, v3 - v1));

            vDataArray[triArray[i * 3 + 0]].norms += N;
            vDataArray[triArray[i * 3 + 1]].norms += N;
            vDataArray[triArray[i * 3 + 2]].norms += N;
        }
        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i].norms = vDataArray[i].norms.normalized;
        }
    }
    void Update()
    {
        for(int i=0; i < speed; i++) {
            computeSpringForce();
            //computeDistanceConstraintForce();
            //computeVolumeConstraintForce();
            globalVolumePreservation();
            UpdateNodes();
        }
        //print("totale volume :"+totalVolume);
        computeVertexNormal();
        vertsBuff.SetData(vDataArray);

        Bounds bounds = new Bounds(Vector3.zero, new Vector3(100, 100, 100));

        // material.SetMatrix("_LocalToWorld", transform.localToWorldMatrix);
        // material.SetMatrix("_WorldToLocal", transform.worldToLocalMatrix);

        material.SetPass(0);
        Graphics.DrawProcedural(
             material,
            bounds,
            MeshTopology.Triangles,
            triArray.Length,
            1,
            null,
            null,
            ShadowCastingMode.On,
            true,
            gameObject.layer
        );

       }
    private float computeTetraVolume(Vector3 i1, Vector3 i2, Vector3 i3, Vector3 i4)
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

    //private void OnGUI()
    //{
    //    int w = Screen.width, h = Screen.height;
    //    GUIStyle style = new GUIStyle();
    //    Rect rect = new Rect(0, 0, w, h * 2 / 100);
    //    style.alignment = TextAnchor.UpperLeft;
    //    style.fontSize = h * 2 / 50;
    //    style.normal.textColor = new Color(1, 1, 1, 1.0f);

    //    float currVolume = 0;
    //    foreach (Tetrahedron t in elements)
    //    {
    //        //currVolume += t.restVolume;
    //        currVolume += computeTetraVolume(Positions[t.i1], Positions[t.i2],
    //            Positions[t.i3], Positions[t.i4]);
    //        //print(t.restVolume);
    //    }
    //    float vLost = (currVolume / totalVolume) * 100.0f;
    //    string text = string.Format("Volume: {0:0.00} %", vLost);
    //    GUI.Label(rect, text, style);

    //    GUIStyle style2 = new GUIStyle();
    //    GUIStyle style3 = new GUIStyle();
    //    Rect rect2 = new Rect(0, 40, w, h * 2 / 100);
    //    style2.alignment = TextAnchor.UpperLeft;
    //    style2.fontSize = h * 2 / 50;
    //    style2.normal.textColor = new Color(1, 1, 1, 1.0f);
    //    string text2 = string.Format("rest Volume: {0:0.0000}", totalVolume);
    //    GUI.Label(rect2, text2, style2);

    //    Rect rect3 = new Rect(0, 80, w, h * 2 / 100);
    //    style3.alignment = TextAnchor.UpperLeft;
    //    style3.fontSize = h * 2 / 50;
    //    style3.normal.textColor = new Color(1, 1, 1, 1.0f);
    //    string text3 = string.Format("Current Volume: {0:0.0000}", currVolume);
    //    GUI.Label(rect3, text3, style3);

    //}

    private void OnDestroy()
    {
        if (this.enabled)
        {
            vertsBuff.Dispose();
            triBuffer.Dispose();
        }
    }
}
