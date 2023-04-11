using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using Assets.Script;
using UnityEngine.Rendering;

public class CPUPBDVolumeconstraint : MonoBehaviour
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
    private string modelName;


    [Header("Obj Parameters")]
    public float mass = 1.0f;
    public float dt = 0.01f; // have to devide by 20
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    public float velocityDamping = -0.1f; 
    public int speed = 1;
    public int iteration = 1;

    [Header("Distance Constrinat Parameters")]
    public float stretchStiffness = 1.0f;
    public float compressStiffness = 1.0f;
    public bool useDistanceConstraint = true;
    [Header("Bending Constrinat Parameters")]
    public float bendingStiffness = 1.0f;
    public bool useBendingConstraint = true;
    [Header("Volume  Constrinat Parameters")]
    public float volumeStiffness = 1.0f;
    public bool useTetVolumeConstraint = true;

    [Header("Collision")]
    public GameObject[] collidableObjects;

    [HideInInspector]
    private int nodeCount;
    private int springCount;
    private int triCount; // size of triangle
    private int tetCount;
    private int bendingCount;
    private int numCollidableSpheres, numCollidableCubes;

    private float invMass;
    //main  property
    //list position
    Vector3[] Positions;
    Vector3[] ProjectPositions;
    Vector3[] WorldPositions;
    Vector3[] Velocities;
    Vector3[] Forces;
    Vector3[] Normals;
    List<Spring> distanceConstraints = new List<Spring>();
    List<Triangle> triangles = new List<Triangle>();
    List<Tetrahedron> tetrahedrons = new List<Tetrahedron>();
    List<Bending> bendingConstraints = new List<Bending>();
    List<SphereCollisionConstraint> sphereCollisionConstraints = new List<SphereCollisionConstraint>();
    List<CubeCollisionConstraint> cubeCollisionConstraints = new List<CubeCollisionConstraint>();
    List<BoxCollisionConstraint> boxCollisionConstraints = new List<BoxCollisionConstraint>();

    Vector3[] DeltaPos;
    int[] deltaCounter;


    bool[] collidedNodes;

    //for render
    ComputeBuffer vertsBuff = null;
    ComputeBuffer triBuffer = null;

    [Header("Rendering Paramenter")]
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private Material material;

    [Header("Label Data")]
    public bool renderVolumeText;
    public string Text;
    public int xOffset;
    public int yOffset;
    public int fontSize;
    public Color textColor = Color.white;
    private Rect rectPos;
    private Color color;

    //public Material material;

    struct vertData
    {
        public Vector3 pos;
        public Vector2 uvs;
        public Vector3 norms;
    };
    int[] triArray;
    vertData[] vDataArray;
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
   
    void addBendingConstraint()
    {
        Dictionary<Edge, List<Triangle>> wingEdges = new Dictionary<Edge, List<Triangle>>(new EdgeComparer());

        // map edges to all of the faces to which they are connected
        foreach (Triangle tri in triangles)
        {
            Edge e1 = new Edge(tri.vertices[0], tri.vertices[1]);
            if (wingEdges.ContainsKey(e1) && !wingEdges[e1].Contains(tri))
            {
                wingEdges[e1].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e1, tris);
            }

            Edge e2 = new Edge(tri.vertices[0], tri.vertices[2]);
            if (wingEdges.ContainsKey(e2) && !wingEdges[e2].Contains(tri))
            {
                wingEdges[e2].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e2, tris);
            }

            Edge e3 = new Edge(tri.vertices[1], tri.vertices[2]);
            if (wingEdges.ContainsKey(e3) && !wingEdges[e3].Contains(tri))
            {
                wingEdges[e3].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e3, tris);
            }
        }

        // wingEdges are edges with 2 occurences,
        // so we need to remove the lower frequency ones
        List<Edge> keyList = wingEdges.Keys.ToList();
        foreach (Edge e in keyList)
        {
            if (wingEdges[e].Count < 2)
            {
                wingEdges.Remove(e);
            }
        }

        bendingCount = wingEdges.Count;

        foreach (Edge wingEdge in wingEdges.Keys)
        {
            /* wingEdges are indexed like in the Bridson,
                * Simulation of Clothing with Folds and Wrinkles paper
                *    3
                *    ^
                * 0  |  1
                *    2
                */

            int[] indices = new int[4];
            indices[2] = wingEdge.startIndex;
            indices[3] = wingEdge.endIndex;

            int b = 0;
            foreach (Triangle tri in wingEdges[wingEdge])
            {
                for (int i = 0; i < 3; i++)
                {
                    int point = tri.vertices[i];
                    if (point != indices[2] && point != indices[3])
                    {
                        //tri #1
                        if (b == 0)
                        {
                            indices[0] = point;
                            break;
                        }
                        //tri #2
                        else if (b == 1)
                        {
                            indices[1] = point;
                            break;
                        }
                    }
                }
                b++;
            }
            Vector3 p0 = Positions[indices[0]];
            Vector3 p1 = Positions[indices[1]];
            Vector3 p2 = Positions[indices[2]];
            Vector3 p3 = Positions[indices[3]];

            Vector3 n1 = (Vector3.Cross(p2 - p0, p3 - p0)).normalized;
            Vector3 n2 = (Vector3.Cross(p3 - p1, p2 - p1)).normalized;

            float d = Vector3.Dot(n1, n2);
            d = Mathf.Clamp(d, -1.0f, 1.0f);

            Bending bending = new Bending();
            bending.index0 = indices[0];
            bending.index1 = indices[1];
            bending.index2 = indices[2];
            bending.index3 = indices[3];
            bending.restAngle = Mathf.Acos(d);

            bendingConstraints.Add(bending);
        }
    }
    private void setupMeshData()
    {
        //LoadModel.LoadData("Torus", obj);
        LoadModel.LoadData(modelName, gameObject);

        Positions = LoadModel.positions.ToArray();
        triangles = LoadModel.faces;
        distanceConstraints = LoadModel.springs;
        triArray = LoadModel.triangles.ToArray();
        //if (useTetVolumeConstraint)
        tetrahedrons = LoadModel.element;

        

        nodeCount = Positions.Length;
        springCount = distanceConstraints.Count;
        triCount = triangles.Count; //
        tetCount = tetrahedrons.Count;

        if (useBendingConstraint)
            addBendingConstraint();

        WorldPositions = new Vector3[nodeCount];
         ProjectPositions = new Vector3[nodeCount];
        Velocities = new Vector3[nodeCount];
        Forces = new Vector3[nodeCount];
        DeltaPos = new Vector3[nodeCount];
        deltaCounter = new int[nodeCount];
        DeltaPos.Initialize();
        deltaCounter.Initialize();

        ProjectPositions = LoadModel.positions.ToArray();
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
    private void setupCollisionConstraint()
    {
        for (int i = 0; i < collidableObjects.Length; i++)
        {
            Collider collider = collidableObjects[i].GetComponent<Collider>();
            if (collider.GetType() == typeof(SphereCollider))
            {
                //add Sphere collider
                SphereCollisionConstraint sphere = new SphereCollisionConstraint();
                sphere.sphereCenter = collider.GetComponent<SphereCollider>().center + collider.transform.position;
                sphere.sphereRadius = collider.GetComponent<SphereCollider>().radius * collider.transform.lossyScale.x;



                sphereCollisionConstraints.Add(sphere);

                print(sphere.sphereCenter);
                print(sphere.sphereRadius);
            }
            else if (collider.GetType() == typeof(BoxCollider))
            {
                //add box collider
                CubeCollisionConstraint cube = new CubeCollisionConstraint();
                cube.cubeExtent = 0.5f * collider.GetComponent<BoxCollider>().size;
                cube.cubeTransform = collider.transform;
                cubeCollisionConstraints.Add(cube);


                //box collider

                //BoxCollisionConstraint box = new BoxCollisionConstraint();
                //Vector3 maxPos = Vector3.zero;
                //Vector3 minPos = Vector3.zero;
                //Vector3 center = Vector3.zero;

                ////use only mesh not cosider box collider for current steps
                //for (int j = 0; j < 3; j++)
                //{
                //    maxPos[j] = collider.transform.position[j] + collider.transform.localScale[j] / 2;
                //    minPos[j] = collider.transform.position[j] - collider.transform.localScale[j] / 2;
                //}
                //box.MaxPos = maxPos;
                //box.MinPos = minPos;
                //box.Center = collider.transform.position;
                //boxCollisionConstraints.Add(box);

                //print("max pos :: " + maxPos);
                //print("min pos :: " + minPos);

            }

        }
        //collidedNodes = new bool[nodeCount];
        //print(boxCollisionConstraints.Count);
    }
    void setup()
    {
        //obj = gameObject;
        material = new Material(renderingShader); // new material for difference object
        material.color = matColor; //set color to material

        SelectModelName();
        setupMeshData();
        setupCollisionConstraint();
        setupShader();
        setBuffData();

        totalVolume = computeObjectVolume();
    }

    void Start()
    {

        setup();
        //print(nodeCount);
        //print(springCount);
        //foreach (Tetrahedron t in tetrahedrons)
        //{
        //    totalVolume += t.RestVolume;
        //}

        //print(transform.localToWorldMatrix);
        //print(transform.worldToLocalMatrix);
    }
    void addExternalForce(Vector3 force)
    {
        for (int i = 0; i < nodeCount; i++)
        {
            Velocities[i] += (force /mass) * dt;
            //cout << to_string(Velocities[i]) << endl;
        }
    }
    void addExplicitEuler()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            ProjectPositions[i] = Positions[i] + Velocities[i] * dt;
            //cout << to_string(ProjectPositions[i]) << endl;
        }
    }

    void collisionDetectionAndRespone()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            //floor position = Vector3(x,-2,z)
            //if (transform.TransformPoint(ProjectPositions[i]).y < -2.0f)
            //{

            //    ProjectPositions[i].y = transform.InverseTransformPoint(
            //        new Vector3(ProjectPositions[i].x, -2.0f, ProjectPositions[i].z)).y;
            //    Velocities[i] = Vector3.zero;
            //}
            if ((ProjectPositions[i]).y < transform.InverseTransformPoint(new Vector3(0,- 2.0f,0)).y)
            {
                ProjectPositions[i].y = transform.InverseTransformPoint(
                    new Vector3(ProjectPositions[i].x, -2.0f, ProjectPositions[i].z)).y;
                Velocities[i] = Vector3.zero;
            }

        }
    }
    void satisfyDistanceConstraint()
    {
        for (int i = 0; i < springCount; i++)
        {
            Spring constraint = distanceConstraints[i];
            int i1 = constraint.i1;
            int i2 = constraint.i2;
            float restLength = constraint.RestLength;

            Vector3 pi = ProjectPositions[i1];
            Vector3 pj = ProjectPositions[i2];

            float d = Vector3.Distance(pi , pj);

            Vector3 n = (pi - pj).normalized;

            float wi = 1; //inverse mass
            float wj = 1; //inverse mass


            float stiffness = d < restLength ? compressStiffness : stretchStiffness;
            Vector3 deltaP1 = stiffness * wi / (wi + wj) * (d - restLength) * n;
            Vector3 deltaP2 = stiffness * wj / (wi + wj) * (d - restLength) * n;

            ProjectPositions[i1] -= deltaP1;
            ProjectPositions[i2] += deltaP2;
        }
    }



    void satisfyDistanceConstraintJacobiStyle()
    {
        //using delta pos to project pos buffer
        for (int i = 0; i < springCount; i++)
        {
            Spring constraint = distanceConstraints[i];
            int i1 = constraint.i1;
            int i2 = constraint.i2;
            float restLength = constraint.RestLength;

            Vector3 pi = ProjectPositions[i1];
            Vector3 pj = ProjectPositions[i2];

            float d = Vector3.Distance(pi, pj);

            Vector3 n = (pi - pj).normalized;

            float wi = 1; //inverse mass
            float wj = 1; //inverse mass


            float stiffness = d < restLength ? compressStiffness : stretchStiffness;
            Vector3 deltaP1 = -stiffness * wi / (wi + wj) * (d - restLength) * n;
            Vector3 deltaP2 = stiffness * wj / (wi + wj) * (d - restLength) * n;

            DeltaPos[i1] += deltaP1;
            DeltaPos[i2] += deltaP2;

            deltaCounter[i1]++;
            deltaCounter[i2]++;

        }

    }
    void satisfyBendingConstraintJacStyle()
    {
        for (int i = 0; i < bendingCount; i++)
        {
            Bending bending = bendingConstraints[i];

            Vector3 p0 = ProjectPositions[bending.index0];
            Vector3 p1 = ProjectPositions[bending.index1];
            Vector3 p2 = ProjectPositions[bending.index2];
            Vector3 p3 = ProjectPositions[bending.index3];

            //Vector3 n1 = (Vector3.Cross(p2 - p0, p3 - p0)).normalized;
            //Vector3 n2 = (Vector3.Cross(p3 - p1, p2 - p1)).normalized;

            //float d = Vector3.Dot(n1, n2);
            //d = Mathf.Clamp(d,- 1.0f, 1.0f);
            //float restAngle = Mathf.Acos(d);

            Vector3 wing = p3 - p2;
            float wingLength = wing.magnitude;

            if (wingLength >= 1e-7)
            {
                Vector3 n1 = Vector3.Cross(p2 - p0, p3 - p0);
                n1 /= n1.sqrMagnitude;

                Vector3 n2 = Vector3.Cross(p3 - p1, p2 - p1);
                n2 /= n2.sqrMagnitude;

                float invWingLength = 1.0f / wingLength;

                Vector3 q0 = wingLength * n1;
                Vector3 q1 = wingLength * n2;
                Vector3 q2 = Vector3.Dot(p0 - p3, wing) * invWingLength * n1
                            + Vector3.Dot(p1 - p3, wing) * invWingLength * n2;
                Vector3 q3 = Vector3.Dot(p2 - p0, wing) * invWingLength * n1
                            + Vector3.Dot(p2 - p1, wing) * invWingLength * n2;

                n1.Normalize();
                n2.Normalize();

                float d = Vector3.Dot(n1, n2);
                d = Mathf.Clamp(d, -1.0f, 1.0f);
                float currentAngle = Mathf.Acos(d);

                float lamda = 0;
                lamda += mass * q0.sqrMagnitude;
                lamda += mass * q1.sqrMagnitude;
                lamda += mass * q2.sqrMagnitude;
                lamda += mass * q3.sqrMagnitude;

                if (lamda != 0.0f)
                {
                    lamda = (currentAngle - bending.restAngle) / lamda * bendingStiffness;

                    if (Vector3.Dot(Vector3.Cross(n1, n2), wing) > 0.0f)
                    {
                        lamda = -lamda;
                    }

                    DeltaPos[bending.index0] -= mass * lamda * q0;
                    DeltaPos[bending.index1] -= mass * lamda * q1;
                    DeltaPos[bending.index2] -= mass * lamda * q2;
                    DeltaPos[bending.index3] -= mass * lamda * q3;

                    deltaCounter[bending.index0]++;
                    deltaCounter[bending.index1]++;
                    deltaCounter[bending.index2]++;
                    deltaCounter[bending.index3]++;


                }
            }
        }
    }
    void satisfyVolumeConstraintJacStyle()
    {
        for (int i = 0; i < tetCount; i++)
        {

            Tetrahedron t = tetrahedrons[i];
            //cout << t.initVolume << endl;

            int idx1 = t.i1;
            int idx2 = t.i2;
            int idx3 = t.i3;
            int idx4 = t.i4;

            Vector3 p0 = ProjectPositions[idx1];
            Vector3 p1 = ProjectPositions[idx2];
            Vector3 p2 = ProjectPositions[idx3];
            Vector3 p3 = ProjectPositions[idx4];

            float volume = computeTetraVolume(p0, p1, p2, p3);

            float restVolume = t.RestVolume;

            Vector3 grad0 = Vector3.Cross(p1 - p2, p3 - p2);
            Vector3 grad1 = Vector3.Cross(p2 - p0, p3 - p0);
            Vector3 grad2 = Vector3.Cross(p0 - p1, p3 - p1);
            Vector3 grad3 = Vector3.Cross(p1 - p0, p2 - p0);

            float lambda = grad0.x * grad0.x + grad0.y * grad0.y + grad0.z * grad0.z +
                grad1.x * grad1.x + grad1.y * grad1.y + grad1.z * grad1.z +
                grad2.x * grad2.x + grad2.y * grad2.y + grad2.z * grad2.z +
                grad3.x * grad3.x + grad3.y * grad3.y + grad3.z * grad3.z;


            lambda = volumeStiffness * (volume - restVolume) / lambda;

            Vector3 deltaP1 = -lambda * grad0;
            Vector3 deltaP2 = -lambda * grad1;
            Vector3 deltaP3 = -lambda * grad2;
            Vector3 deltaP4 = -lambda * grad3;

            DeltaPos[idx1] += deltaP1;
            DeltaPos[idx2] += deltaP2;
            DeltaPos[idx3] += deltaP3;
            DeltaPos[idx4] += deltaP4;

            deltaCounter[idx1]++;
            deltaCounter[idx2]++;
            deltaCounter[idx3]++;
            deltaCounter[idx4]++;

        }
    }

    void averageConstraintDelta()
    {
        //average constraint delta
        for (int i = 0; i < nodeCount; i++)
        {
            DeltaPos[i] /= deltaCounter[i];

            ProjectPositions[i] += DeltaPos[i] * 1.5f;
            DeltaPos[i] = Vector3.zero;
            deltaCounter[i] = 0;
        }
    }


    void satisfyBendingConstraint()
    {
        for (int i = 0; i < bendingCount; i++)
        {
            Bending bending = bendingConstraints[i];

            Vector3 p0 = ProjectPositions[bending.index0];
            Vector3 p1 = ProjectPositions[bending.index1];
            Vector3 p2 = ProjectPositions[bending.index2];
            Vector3 p3 = ProjectPositions[bending.index3];

            //Vector3 n1 = (Vector3.Cross(p2 - p0, p3 - p0)).normalized;
            //Vector3 n2 = (Vector3.Cross(p3 - p1, p2 - p1)).normalized;

            //float d = Vector3.Dot(n1, n2);
            //d = Mathf.Clamp(d,- 1.0f, 1.0f);
            //float restAngle = Mathf.Acos(d);

            Vector3 wing = p3 - p2;
            float wingLength = wing.magnitude;

            if (wingLength >= 1e-7)
            {
                Vector3 n1 = Vector3.Cross(p2 - p0, p3 - p0);
                n1 /= n1.sqrMagnitude;

                Vector3 n2 = Vector3.Cross(p3 - p1, p2 - p1);
                n2 /= n2.sqrMagnitude;

                float invWingLength = 1.0f / wingLength;

                Vector3 q0 = wingLength * n1;
                Vector3 q1 = wingLength * n2;
                Vector3 q2 = Vector3.Dot(p0 - p3, wing) * invWingLength * n1
                            + Vector3.Dot(p1 - p3, wing) * invWingLength * n2;
                Vector3 q3 = Vector3.Dot(p2 - p0, wing) * invWingLength * n1
                            + Vector3.Dot(p2 - p1, wing) * invWingLength * n2;

                n1.Normalize();
                n2.Normalize();

                float d = Vector3.Dot(n1, n2);
                d = Mathf.Clamp(d, -1.0f, 1.0f);
                float currentAngle = Mathf.Acos(d);

                float lamda = 0;
                lamda += mass * q0.sqrMagnitude;
                lamda += mass * q1.sqrMagnitude;
                lamda += mass * q2.sqrMagnitude;
                lamda += mass * q3.sqrMagnitude;

                if (lamda != 0.0f)
                {
                    lamda = (currentAngle -bending.restAngle) / lamda * bendingStiffness;

                    if (Vector3.Dot(Vector3.Cross(n1, n2), wing) > 0.0f)
                    {
                        lamda = -lamda;
                    }

                    ProjectPositions[bending.index0] -= mass * lamda * q0;
                    ProjectPositions[bending.index1] -= mass * lamda * q1;
                    ProjectPositions[bending.index2] -= mass * lamda * q2;
                    ProjectPositions[bending.index3] -= mass * lamda * q3;
                }
            }
        }
    }
    void satisfyVolumeConstraint()
    {
        for (int i = 0; i < tetCount; i++)
        {

            Tetrahedron t = tetrahedrons[i];
            //cout << t.initVolume << endl;

            int idx1 = t.i1;
            int idx2 = t.i2;
            int idx3 = t.i3;
            int idx4 = t.i4;

           Vector3 p0 = ProjectPositions[idx1];
           Vector3 p1 = ProjectPositions[idx2];
           Vector3 p2 = ProjectPositions[idx3];
           Vector3 p3 = ProjectPositions[idx4];

            float volume = computeTetraVolume(p0, p1, p2, p3);

            float restVolume = t.RestVolume;

            Vector3 grad0 = Vector3.Cross(p1 - p2, p3 - p2);
            Vector3 grad1 = Vector3.Cross(p2 - p0, p3 - p0);
            Vector3 grad2 = Vector3.Cross(p0 - p1, p3 - p1);
            Vector3 grad3 = Vector3.Cross(p1 - p0, p2 - p0);

            float lambda = grad0.x * grad0.x + grad0.y * grad0.y + grad0.z * grad0.z +
                grad1.x * grad1.x + grad1.y * grad1.y + grad1.z * grad1.z +
                grad2.x * grad2.x + grad2.y * grad2.y + grad2.z * grad2.z +
                grad3.x * grad3.x + grad3.y * grad3.y + grad3.z * grad3.z;


            lambda = volumeStiffness * (volume - restVolume) / lambda;

           Vector3 deltaP1 = -lambda * grad0;
           Vector3 deltaP2 = -lambda * grad1;
           Vector3 deltaP3 = -lambda * grad2;
           Vector3 deltaP4 = -lambda * grad3;

            ProjectPositions[idx1] += deltaP1;
            ProjectPositions[idx2] += deltaP2;
            ProjectPositions[idx3] += deltaP3;
            ProjectPositions[idx4] += deltaP4;

        }
    }
    void satisfyCubeCollision()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            foreach (CubeCollisionConstraint cube in cubeCollisionConstraints)
            {
                //Vector3 projectPos = transform.TransformPoint(ProjectPositions[i]);
                Vector3 projectPos = transform.TransformPoint(ProjectPositions[i]);
                Vector3 localPosition = cube.cubeTransform.InverseTransformPoint(projectPos);

                if (cube.IsPointInCube(localPosition))
                {

                    //print("collided :: " + i);
                    //collidedNodes[i] = true;
                    int closestAxis = 0;
                    float closestDist = float.MaxValue;
                    for (int j = 0; j < 3; j++)
                    {
                        float dist = Mathf.Abs(localPosition[j] - cube.cubeExtent[j]);
                        if (dist < closestDist)
                        {
                            closestDist = dist;
                            closestAxis = j;
                        }
                    }
                    float[] newPos = new float[3];

                    for (int j = 0; j < 3; j++)
                    {
                        if (j == closestAxis)
                        {
                            newPos[j] = (cube.cubeExtent[j] + 0.001f) * Mathf.Sign(localPosition[j]);
                        }
                        else
                        {
                            newPos[j] = localPosition[j];
                        }
                    }
                    Vector3 closestPos = new Vector3(newPos[0], newPos[1], newPos[2]);
                    closestPos = cube.cubeTransform.TransformPoint(closestPos);
                    ProjectPositions[i] = (closestPos);
                    //Velocities[i] *= -0.9f;
                    //Velocities[i] = Vector3.zero;
                }
               
            }
        }
    }

    void satisfyBoxCollision()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            foreach (BoxCollisionConstraint box in boxCollisionConstraints)
            {
                //check if point collide in box
                Vector3 localProjPos = transform.TransformPoint(ProjectPositions[i]);

                if (box.IsCollided(localProjPos))
                {
                    //Vector3 newProjPos = transform.InverseTransformPoint(localProjPos);
                    Vector3 newProjPos = localProjPos;
                    //xAxis
                    //if(Mathf.Abs(localProjPos.x - box.MaxPos.x) < 
                    //    Mathf.Abs(localProjPos.x - box.MinPos.x))
                    //{
                    //    newProjPos.x = box.MaxPos.x;
                    //}
                    //else
                    //{
                    //    newProjPos.x = box.MinPos.x;
                    //}

                    if(localProjPos.x >= box.MinPos.x && localProjPos.x <= box.MaxPos.x)
                        newProjPos.x = (Mathf.Abs(localProjPos.x - box.MaxPos.x) < 
                        Mathf.Abs(localProjPos.x - box.MinPos.x)) ? 
                        box.MaxPos.x+0.001f : box.MinPos.x + 0.001f;
                    if (localProjPos.y >= box.MinPos.y && localProjPos.y <= box.MaxPos.y)
                        newProjPos.y = (Mathf.Abs(localProjPos.y - box.MaxPos.y) <
                       Mathf.Abs(localProjPos.y - box.MinPos.y)) ?
                       box.MaxPos.y + 0.001f : box.MinPos.y + 0.001f;
                    if (localProjPos.z >= box.MinPos.z && localProjPos.z <= box.MaxPos.z)
                        newProjPos.z = (Mathf.Abs(localProjPos.z - box.MaxPos.z) <
                       Mathf.Abs(localProjPos.z - box.MinPos.z)) ?
                       box.MaxPos.z + 0.001f : box.MinPos.z + 0.001f;

                    //yAxis
                    //zAxis


                    ProjectPositions[i] = transform.InverseTransformPoint(newProjPos);
                    Velocities[i] = Vector3.zero;
                    //print("collided ::"+i);
                }

            }
        }
        
    }
    void satisfySphereCollision()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            Vector3 projectedPosition = transform.TransformPoint(ProjectPositions[i]);
            Vector3 position = transform.TransformPoint(Positions[i]);
            foreach (SphereCollisionConstraint sphere in sphereCollisionConstraints)
            {
                Vector3 direction = (projectedPosition - position).normalized;
                float L = (sphere.sphereCenter - position).magnitude;
                float tc = Vector3.Dot(sphere.sphereCenter - position, direction);
                float d = Mathf.Sqrt(L * L - tc * tc);
                float tc1 = Mathf.Sqrt(sphere.sphereRadius * sphere.sphereRadius - d * d);
                float t = tc - tc1;
                Vector3 collisionPosition = position + direction * t;
                Vector3 collisionNormal = (collisionPosition - sphere.sphereCenter).normalized;


                Vector3 p = projectedPosition;
                float cp = Vector3.Dot(p - collisionPosition, collisionNormal);
                if (cp < 0)
                {
                    Vector3 n = (p - sphere.sphereCenter).normalized;
                    Vector3 q = sphere.sphereCenter + n * (sphere.sphereRadius + 0.01f);

                    ProjectPositions[i] = transform.InverseTransformPoint(q);
                    Velocities[i] *= -1.0f;
                }
            }
        }
    }
    void updatePositions()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            Velocities[i] = (ProjectPositions[i] - Positions[i]) / dt;
            Positions[i] = ProjectPositions[i];
            vDataArray[i].pos = Positions[i];
        }
    }

    void PBDSolving()
    {
        addExternalForce(gravity);
        //damp velocity ****TODO******
        addExplicitEuler();

        for (int j = 0; j < iteration; j++)
        {

            ////old solving style
            //satisfyDistanceConstraint();
            //satisfyBendingConstraint();
            //satisfyVolumeConstraint();

            //new solving style to compare with GPU since GPU is much easily implement using this method
            satisfyDistanceConstraintJacobiStyle();
            satisfyBendingConstraintJacStyle();
            satisfyVolumeConstraintJacStyle();
            averageConstraintDelta();


            collisionDetectionAndRespone();
            //solve collisiion constraint
        }
        updatePositions();
        //apply friction
    }


    void computeVertexNormal()
    {
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
        PBDSolving();
        computeVertexNormal();
        vertsBuff.SetData(vDataArray);

        Bounds bounds = new Bounds(Vector3.zero, Vector3.one*100);

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
    //    foreach (Tetrahedron t in tetrahedrons)
    //    {
    //        //currVolume += t.restVolume;
    //        currVolume += computeTetraVolume(Positions[t.i1], Positions[t.i2],
    //            Positions[t.i3], Positions[t.i4]);
    //        //print(t.restVolume);
    //    }
    //    float vLost = (currVolume / totalVolume) * 100.0f;
    //    string text = string.Format("Volume: {0:0.00} %", vLost);
    //    GUI.Label(rect, text, style);


    //}
    float computeObjectVolume()
    {
        //made by sum of all tetra
        float volume = 0.0f;
        foreach (Tetrahedron tet in tetrahedrons)
        {
            Vector3 p0 = Positions[tet.i1];
            Vector3 p1 = Positions[tet.i2];
            Vector3 p2 = Positions[tet.i3];
            Vector3 p3 = Positions[tet.i4];
            volume += computeTetraVolume(p0,p1,p2,p3);
        }
        return volume;
    }
    private void OnGUI()
    {
        if (renderVolumeText)
        {
            int w = Screen.width, h = Screen.height;
            GUIStyle style = new GUIStyle();

            rectPos = new Rect(0 + xOffset, yOffset, w, h * 2 / 100);
            Rect rect = rectPos;
            style.alignment = TextAnchor.UpperLeft;
            style.fontSize = h * 2 / 50;
            Color col;
            string htmlValue = "#FFED00";
            if (ColorUtility.TryParseHtmlString(htmlValue, out col))
                style.normal.textColor = col;

            //get volume data;

            float currVolume = 0;
            currVolume = computeObjectVolume();

            float vLost;
            if (totalVolume == 0.0f)
            {
                vLost = 0.0f;
            }
            else
            {
                vLost = (currVolume / totalVolume) * 100.0f;
            }
            string text = string.Format("Volume: {0:0.00} %", vLost);
            GUI.Label(rect, text, style);
        }
    }



    //private void OnDrawGizmos()
    //{
    //    if (collidedNodes == null) return;

    //    Gizmos.color = Color.yellow;
    //    for(int i = 0;i<nodeCount;i++)
    //    {
    //        if (collidedNodes[i])
    //            Gizmos.DrawSphere(Positions[i], 0.1f);
    //    }
    //}

    private void OnDestroy()
    {
        vertsBuff.Dispose();
        triBuffer.Dispose();
    }
}
