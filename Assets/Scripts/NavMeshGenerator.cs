using System.Collections.Generic;
using Unity.VisualScripting.Dependencies.Sqlite;
using UnityEngine;

[RequireComponent(typeof(BoxCollider))]
public class NavMeshGenerator : MonoBehaviour
{
#region Params
    List<MeshFilter> geometries = new List<MeshFilter>(); 

    [SerializeField]
    LayerMask navMeshMask;

    [SerializeField]
    float cellSize = 1.0f;

    [SerializeField]
    float cellHeight = 1.0f;

    [SerializeField]
    float maxTraversableAngle = 45.0f;

    [SerializeField]
    float minTraversableHeight = 0.5f;

    [SerializeField]
    float maxTraversableStep = 1.0f;

	//Closest distance any part of a mesh can get to an obstruction in the source geometry
    [SerializeField]
    int traversableAreaBorderSize = 1;

	//Minimum span size of the region to merge
    [SerializeField]
    [Tooltip("The minimum size a region can have to be considered for merging with adjacent regions")]
    int minMergeRegionSize = 1;

	//Minimum span size of the island region to remove
    [SerializeField]
    int minUnconnectedRegionSize = 1;

    [SerializeField]
    bool useConservativeExpansion = true;

    //the maximum distance the edge of the contour can deviate from the source geometry
    //the less, the more acurate the navmesh becomes, but also more expensive to calculate
    [SerializeField]
    float edgeMaxDeviation = 1.0f;

    //the maximum length an edge of a polygon can have in the navmesh borders
    [SerializeField]
    float maxEdgeLength = 1.0f;

#endregion
   
#region GenerationTemporaries
    BoxCollider boundsCollider;

    Bounds navMeshBounds;

    float inverseCellSize;
    float inverseCellHeight;
    int xSize = 0, ySize = 0, zSize = 0;

    HeightSpan[] heightSpans;

    //open span generation
    OpenSpan[] openSpans;
    Region[] regions;
    List<ContourVertexData> simplifiedVertices;
    int minBorderDistance = 0;
    int maxBorderDistance = 0;
    int regionCount = 0;
    int maxDistanceToBorder = 0;

    static Vector2[] offsets = new Vector2[]{ Vector2.left, Vector2.down, Vector2.right, Vector2.up};

#endregion

#region DEBUG

    [SerializeField]
    float debugDuration = 20.0f;

    [SerializeField]
    bool d1, d2, d3, d4, drawHeightSpanGrid, drawOpenSpanGrid, drawDistanceNumbers, drawRegions, drawRegionContours;

    int id = 0;

    Color[] colors = new Color[]
    {
        Color.red,
        Color.black,
        Color.green,
        Color.white,
        Color.yellow,
        Color.blue,
        Color.magenta,
        Color.cyan
    };
#endregion DEBUG
    
#region Generation

    public void Rebuild()
    {
        id = 0;
        boundsCollider = GetComponent<BoxCollider>();

        ResetTemporaries();
        GatherGeometry();
        Generate();
    }

    void ResetTemporaries()
    {
        navMeshBounds = boundsCollider.bounds;
        inverseCellSize = 1.0f / cellSize;
        inverseCellHeight = 1.0f / cellHeight;
        minBorderDistance = 0;
        maxBorderDistance = 0;
        regionCount = 0;
        CalculateGridSize();

        heightSpans = new HeightSpan[xSize * zSize];
        openSpans = new OpenSpan[xSize * zSize];
        simplifiedVertices = new List<ContourVertexData>();
    }
//todo: restrict this to bounds or something
    void GatherGeometry()
    {
        geometries.Clear();

        MeshFilter[] meshesInScene = FindObjectsOfType<MeshFilter>();

        foreach(var mesh in meshesInScene)
        {
            if(navMeshMask.Contains(mesh.gameObject.layer))
            {
                geometries.Add(mesh);
            }
        }
    }

    void CalculateGridSize()
    {
        Vector3 Extent = navMeshBounds.max - navMeshBounds.min;
        xSize = Mathf.CeilToInt(Extent.x * inverseCellSize);
        ySize = Mathf.CeilToInt(Extent.y * inverseCellHeight);
        zSize = Mathf.CeilToInt(Extent.z * inverseCellSize);
    }

    int GetGridIndex(int x, int z)
    {
        return z * xSize + x;
    }

    void Generate()
    {
        if(geometries.Count == 0)
        {
            Debug.Log("No valid geometries found");
            return;
        }

        foreach (var mesh in geometries)
            CreateSolidHeightField(mesh);    

        if(drawHeightSpanGrid)
            DrawDebugHeightSpanData();

        CreateOpenHeightField();

        GenerateContour();
    }

#endregion

#region SolidHeightFieldGeneration
    void CreateSolidHeightField(MeshFilter mesh)
    {
        Mesh sharedMesh = mesh.sharedMesh;

        //get the vertices from the mesh and transform them, using the mesh gameobjects transformation
        Vector3[] vertices = new Vector3[sharedMesh.vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 transformedVertex = mesh.transform.TransformPoint(sharedMesh.vertices[i]);
            vertices[i] = transformedVertex;

        }
        var indices = sharedMesh.GetIndices(0);

        Voxelize(vertices, indices);
        MarkLowHeightSpan();
        MarkLedgeSpan();
    }

    void Voxelize(Vector3[] vertices, int[] indices)
    {
        FindGeometryHeight(vertices, out float solidFieldMinHeight, out float solidFieldMaxHeight);

        int polyCount = indices.Length / 3;

        //for every polygon
        for (int polyIndex = 0; polyIndex < polyCount; polyIndex++)
        {
            //Find the vertices coordinates for every triangle of the mesh and add them to the array
            Vector3[] polyVertices = new Vector3[]
            {
                vertices[indices[polyIndex * 3]],
                vertices[indices[polyIndex * 3 + 1]],
                vertices[indices[polyIndex * 3 + 2]]
            };

            bool walkable = FilterWalkablePolygon(polyVertices);

		    //Find the bounding box surrounding the triangle by comparing the vertices coordinates
            Vector3 triBoundsMin = polyVertices[0], triBoundsMax = polyVertices[0];
            for (int i = 1; i < 3; i++)
            {
                triBoundsMin.x = Mathf.Min(triBoundsMin.x, polyVertices[i].x);
                triBoundsMin.y = Mathf.Min(triBoundsMin.y, polyVertices[i].y);
                triBoundsMin.z = Mathf.Min(triBoundsMin.z, polyVertices[i].z);

                triBoundsMax.x = Mathf.Max(triBoundsMax.x, polyVertices[i].x);
                triBoundsMax.y = Mathf.Max(triBoundsMax.y, polyVertices[i].y);
                triBoundsMax.z = Mathf.Max(triBoundsMax.z, polyVertices[i].z);
            }

            //Draw Debug
            if(d1)
                DrawMinMaxBox(triBoundsMin, triBoundsMax, Color.blue, debugDuration);
            
            if(d2)
                DrawMeshFaces(polyVertices, Color.green, debugDuration);

            //cell bounding box
            //the minimum cell and maximum cell this triangle touches
            int xMin = (int)((triBoundsMin.x - navMeshBounds.min.x) * inverseCellSize);
            int yMin = (int)((triBoundsMin.y - navMeshBounds.min.y) * inverseCellHeight);
            int zMin = (int)((triBoundsMin.z - navMeshBounds.min.z) * inverseCellSize);
            int xMax = (int)((triBoundsMax.x - navMeshBounds.min.x) * inverseCellSize);
            int yMax = (int)((triBoundsMax.y - navMeshBounds.min.y) * inverseCellHeight);
            int zMax = (int)((triBoundsMax.z - navMeshBounds.min.z) * inverseCellSize);

            xMin = Mathf.Clamp(xMin, 0, xSize - 1);
            yMin = Mathf.Clamp(yMin, 0, ySize - 1);
            zMin = Mathf.Clamp(zMin, 0, zSize - 1);
            xMax = Mathf.Clamp(xMax, 0, xSize - 1);
            yMax = Mathf.Clamp(yMax, 0, ySize - 1);
            zMax = Mathf.Clamp(zMax, 0, zSize - 1);

		    //The height extension of the heightfield
		    float fieldHeight = solidFieldMaxHeight - navMeshBounds.min.z;

            //Debug.Log("MinCell" + xMin + "|" + yMin + "|" + zMin);
            //Debug.Log("MaxCell" + xMax + "|" + yMax + "|" + zMax);

            //go through all cells that this polygon occupies
            for (int x = xMin; x <= xMax; x++)
            {
                for (int z = zMin; z <= zMax; z++, id++)
                {
                    //minimum and maximum coordinates for the current cell
                    Vector3 cellMinCoord = new Vector3(navMeshBounds.min.x + cellSize * x, solidFieldMinHeight, navMeshBounds.min.z + cellSize * z);
				    Vector3 cellMaxCoord = new Vector3(cellMinCoord.x + cellSize, solidFieldMaxHeight, cellMinCoord.z + cellSize);

                    List<Vector3> clippedPolygonVertices = new List<Vector3>(polyVertices);

                    //create a clipped polyon inside the cell
                    ClipPolygon(ref clippedPolygonVertices, cellMinCoord, cellMaxCoord);

                    int clippedVertices = clippedPolygonVertices.Count;
                    //Debug.Log(clippedVertices);

                    if(clippedVertices < 3)
                        continue;

                    if(d4)
                        DrawPolygon(clippedPolygonVertices, colors[id%colors.Length], 20.0f);

                    float heightMin = clippedPolygonVertices[0].y;
                    float heightMax = clippedPolygonVertices[0].y;

                    for (int it = 1; it < clippedVertices; it++)
                    {
                        heightMin = Mathf.Min(heightMin, clippedPolygonVertices[it].y);
                        heightMax = Mathf.Max(heightMax, clippedPolygonVertices[it].y);
                    }

                    // Convert to height above the base of the heightfield.
                    heightMin -= navMeshBounds.min.y;
                    heightMax -= navMeshBounds.min.y;

                    // The height of the cell is entirely outside the bounds of the heightfield, skip the cell
                    if (heightMax < 0.0f || heightMin > fieldHeight)
                        continue;
                    

                    //Make sure the height min and max are clamped to the bound of the bounding box  
                    if (heightMin < 0.0f)
                        heightMin = 0.0f;

                    if (heightMax > fieldHeight)
                        heightMax = fieldHeight;
                    
                    //Convert the height coordinate data to voxel/grid data
                    int heightIndexMin =  Mathf.Clamp(Mathf.FloorToInt(heightMin * inverseCellHeight), 0, int.MaxValue);
                    int heightIndexMax =  Mathf.Clamp(Mathf.CeilToInt(heightMax * inverseCellHeight), 0, int.MaxValue);

                    //Debug.Log("Min" + heightIndexMin);
                    //Debug.Log("Max" + heightIndexMax);
                    AddSpanData(x, heightIndexMin, heightIndexMax, z, walkable);

                    //Draw debug info relative to single valid cells
                    for (int i = heightIndexMin; i < heightIndexMax; ++i)
                    {
                        Vector3 cellMinDebug = new Vector3(cellMinCoord.x, navMeshBounds.min.y + cellHeight * i, cellMinCoord.z);
                        Vector3 cellMaxDebug = new Vector3(cellMaxCoord.x, navMeshBounds.min.y + cellHeight * i + cellHeight, cellMaxCoord.z);

                        if(d3)
                            DrawMinMaxBox(cellMinDebug, cellMaxDebug, Color.green, 20.0f);
                    }
                }
            }

        }
    }

    void FindGeometryHeight(Vector3[] vertices, out float minHeight, out float maxHeight)
    {
        //Assign the bound min and max to the coordinates of the first vertex
        minHeight = vertices[0].y;
        maxHeight = vertices[0].y;

        //Iterate through all the vertices to find the actual bounds
        foreach (var vertex in vertices)
        {
            minHeight = Mathf.Min(vertex.y, minHeight);
            maxHeight = Mathf.Max(vertex.y, maxHeight);
        }
    }

    //return false if unwalkable, true if walkable
    bool FilterWalkablePolygon(Vector3[] vertices)
    {
        //Debug.Log("0"+ vertices[0]);
        //Debug.Log("1"+ vertices[1]);
        //Debug.Log("2"+ vertices[2]);

        Vector3 diffAB = vertices[1] - vertices[0];
	    Vector3 diffAC = vertices[2] - vertices[0];
        diffAB.Normalize();
        diffAC.Normalize();
        Vector3 result = Vector3.Cross(diffAB, diffAC);
        result.Normalize();
        float dot = Vector3.Dot(result, Vector3.up);
        float angle = Mathf.Acos(dot) * Mathf.Rad2Deg;
        //Debug.Log("R" + result);
        //Debug.Log(dot);
        //Debug.Log(angle);
        //if(angle < maxTraversableAngle)
        //    Debug.Log("Walkable");
        return angle < maxTraversableAngle;
    }

    void ClipPolygon(ref List<Vector3> vertices, Vector3 minBounds, Vector3 maxBounds)
    {   
        if(vertices.Count == 0)
            return;
        
        ClipVersusPlane(ref vertices, minBounds, maxBounds, Side.X_N);
        ClipVersusPlane(ref vertices, minBounds, maxBounds, Side.X_P);
        ClipVersusPlane(ref vertices, minBounds, maxBounds, Side.Y_P);
        ClipVersusPlane(ref vertices, minBounds, maxBounds, Side.Y_N);
        ClipVersusPlane(ref vertices, minBounds, maxBounds, Side.Z_P);
        ClipVersusPlane(ref vertices, minBounds, maxBounds, Side.Z_N);

        return;
        
        //for (Side side = Side.X_N; side < Side.Max; side++)
        //    ClipVersusPlane(ref vertices, minBounds, maxBounds, side);
    }

    void ClipVersusPlane(ref List<Vector3> vertices, Vector3 minBounds, Vector3 maxBounds, Side side)
    {
        if(vertices.Count < 3)
        {
            //Debug.LogWarning("Insufficient number of vertices passed to ClipPolygon()");
            return;
        }

        List<Vector3> verticesOut = new List<Vector3>();

        Vector3 v0;
        Vector3 v1 = vertices[vertices.Count - 1];
        for (int i = 0; i < vertices.Count; i++)
        {
            v0 = vertices[i];

            //Sutherland-Hodgeman Algorithm
            //Check if the first point is inside the clipping plane 
		    if (InsideClippingPlane(v0, minBounds, maxBounds, side))
		    {
                //Check if the second point is outside the clipping plane
                if (!InsideClippingPlane(v1, minBounds, maxBounds, side))
                {
                    //The second point is outside while the first is inside:
                    //The intersection point of the line defining the clipping plane should be included in the output, followed by the first point
                    Vector3 intersection = FindPlaneIntersection(v0, v1, minBounds, maxBounds, side);
                    verticesOut.Add(intersection);
                }
                //else
                //{
                    //If both of the points are inside, only the first one is added to the output
                    verticesOut.Add(v0);
                //}
		    }
		    //If the first point is outside, check if the second one is inside
		    else if (InsideClippingPlane(v1, minBounds, maxBounds, side))
		    {
                //If it is the intersection of the line defining the clipping plane should be included in the output
                Vector3 intersection = FindPlaneIntersection(v1, v0, minBounds, maxBounds, side);
                verticesOut.Add(intersection);
		    }

		    //If both of the points are outside, nothing is added to the output polygon
		    v1 = v0;
        }

        vertices = verticesOut;
    }

    bool InsideClippingPlane(Vector3 vertex, Vector3 minBounds, Vector3 maxBounds, Side side)
    {
        switch (side)
        {
        case Side.X_N: return vertex.x >= minBounds.x;
        case Side.X_P: return vertex.x <= maxBounds.x;
        case Side.Y_N: return vertex.y >= minBounds.y;
        case Side.Y_P: return vertex.y <= maxBounds.y;
        case Side.Z_N: return vertex.z >= minBounds.z;
        case Side.Z_P: return vertex.z <= maxBounds.z;
        }
        return false;
    }

    Vector3 FindPlaneIntersection(Vector3 v0, Vector3 v1, Vector3 minBounds, Vector3 maxBounds, Side side)
    {
	    Vector3 planeOrigin = Vector3.zero;
        Vector3 axis = Vector3.zero;
        Vector3 center = (maxBounds + minBounds) / 2.0f;

        //Retrieve the origin coordinates of every plane based on the side considered and find the intersection point
        //The normals can be easily found because the box is axis aligned
        switch (side)
        {
        case Side.X_N:
            planeOrigin = new Vector3(minBounds.x, center.y, center.z);
            axis = Vector3.left;
            break;

        case Side.X_P:
            planeOrigin = new Vector3(maxBounds.x, center.y, center.z);
            axis = Vector3.right;
            break;

        case Side.Y_N:
            planeOrigin = new Vector3(center.x, minBounds.y, center.z);
            axis = Vector3.down;
            break;

        case Side.Y_P:
            planeOrigin = new Vector3(center.x, maxBounds.y, center.z);
            axis = Vector3.up;
            break;

        case Side.Z_N:
            planeOrigin =new Vector3(center.x, center.y, minBounds.z);
            axis = Vector3.back;
            break;

        case Side.Z_P:
            planeOrigin = new Vector3(center.x, center.y, maxBounds.z);
            axis = Vector3.forward;
            break;
        }
        Vector3 intersection = Math.LinePlaneIntersection(v0, v1, planeOrigin, axis);
        return intersection;
    }

    bool AddSpanData(int x, int yMin, int yMax, int z, bool walkable)
    {
        HeightSpan newSpan = new HeightSpan(x, yMin, yMax, z, walkable);
        int index = GetGridIndex(x,z);

        //If the grid location contains no data, generate a new span and add it to the container
        if(heightSpans[index] == null)
        {
            heightSpans[index] = newSpan;
            return true;
        }

        HeightSpan currentSpan = heightSpans[index];
        HeightSpan previousSpan = null;

        while(currentSpan != null)
        {
	
		    //Check if the new span is below the current span
            if (currentSpan.yMin > yMax + 1)
            {
                //If it is, create a new span and insert it below the current span
                newSpan.nextSpan = currentSpan;

                //If the new span is the first one in this column, insert it at the base
                if (previousSpan == null)
                {
                    heightSpans[index] = newSpan;
                }
                //If the new span is between 2 spans, link the previous span to the new one
                else
                {
                    previousSpan.nextSpan = newSpan;
                }

                return true;
            }

            //Current span is below the new span
            if (currentSpan.yMax < yMin - 1)
            {
                //Current span is not adjacent to new span
                if (currentSpan.nextSpan == null)
                {
                    //Locate the new span above the current one
                    currentSpan.nextSpan = newSpan;

                    return true;
                }

                previousSpan = currentSpan;
                currentSpan = currentSpan.nextSpan;
            }

            //There's overlap or adjacency between new and current span, merge is needed
            else
            {
                if (yMin < currentSpan.yMin)
                {
                    //Base on the condition above, set the new height min of the current span
                    currentSpan.yMin = yMin;
                }

                if (yMax == currentSpan.yMax)
                {
                    //Base on the condition above, merge the span type
                    currentSpan.walkable = walkable;
                    return true;
                }

                if (currentSpan.yMax > yMax)
                {
                    //Current span is higher than new one, no need to preform any action, current one takes priority
                    return true;
                }

                //If all the condition above are skipped, the new spans's maximum height is higher than the current span's maximum height
                //Need to check where the merge ends
                HeightSpan nextSpan = currentSpan.nextSpan;
                while (true)
                {		
                    if (nextSpan == null || nextSpan.yMin > yMax + 1)
                    {
                        //If there are no spans above the current one or the height increase does not affect the next span
                        //the current span max and type can be directly replaced and set
                        currentSpan.yMax = yMax;
                        currentSpan.walkable = walkable;

                        //If current span at top of the column, remove any possible link it could have had
                        if (nextSpan == null)
                        {
                            currentSpan.nextSpan = null;
                        }
                        else
                        {
                            currentSpan.nextSpan = nextSpan;
                        }

                        return true;
                    }

                    //The new height of the current span is overlapping with another span, merging needed
                    //If no gap between current and next span and no overlap as well
                    if (nextSpan.yMin == yMax + 1 || yMax <= nextSpan.yMax)
                    {
                        currentSpan.yMax = nextSpan.yMax;
                        currentSpan.nextSpan = nextSpan.nextSpan;
                        currentSpan.walkable = nextSpan.walkable;

                        //If the new span has the same height of the current ne, merge the attribute
                        if (yMax == currentSpan.yMax)
                        {
                            currentSpan.walkable = walkable;
                            return true;
                        }

                        return true;
                    }

                    //The current span overlaps the next one, go up in the column to see when the next will be fully included
                    nextSpan = nextSpan.nextSpan;
                }
            }
        }

        return false;
    }

    void MarkLowHeightSpan()
    {
        //Iterate through all the base span
        foreach (var span in heightSpans)
        {
            //As long as the current span has a next span valid
            HeightSpan currentSpan = span;
            if(currentSpan == null)
                continue;
            do 
            {
                //If already unwalkable, skip
                if (!currentSpan.walkable)
                {
                    currentSpan = currentSpan.nextSpan;
                    continue;
                }

                //Find the height distance between the current and next span, if less than the MinTraversableHeight flag the current span as unwalkable
                int spanFloor = currentSpan.yMax;
                int spanCeiling = (currentSpan.nextSpan != null) ? currentSpan.nextSpan.yMin : int.MaxValue;

                if ((spanCeiling - spanFloor) * cellHeight <= minTraversableHeight)
                    currentSpan.walkable = false;

                //To iterate through all the spans, let the current span become the next and repeat the loop until valid
                currentSpan = currentSpan.nextSpan;
            } 
            while (currentSpan != null);
        }
    }

    void MarkLedgeSpan()
    {
        //Iterate through all the base span
        foreach (var span in heightSpans)
        {
            //As long as the current span has a next span valid
            HeightSpan currentSpan = span;

            while(currentSpan != null)
            {
                //If already unwalkable, skip
                if (!currentSpan.walkable)
                {
                    currentSpan = currentSpan.nextSpan;
                    continue;
                }

                int currentFloor = (int)(currentSpan.yMax * cellHeight);
                int currentCeiling = (currentSpan.nextSpan != null) ? (int)(currentSpan.nextSpan.yMin * cellHeight) : int.MaxValue;

                //Minimum height distance from a neightbor span in unit 
                int minHeightToNeightbor = int.MaxValue;

                //Find all the adjacent grid column to the one the span considered is in 
                for (int neightborDir = 0; neightborDir < 4; neightborDir++)
                {
                    int nX = currentSpan.x + (int)offsets[neightborDir].x;
                    int nZ = currentSpan.z + (int)offsets[neightborDir].y;

                    int neighborIndex = GetGridIndex(nX, nZ);

                    if(neighborIndex < 0 || neighborIndex >= xSize * zSize)
                        continue;

                    //If one of the neightbor is not valid, the span considered is on a edge, therefore is not walkable
                    if (heightSpans[neighborIndex] == null)
                    {
                        minHeightToNeightbor = Mathf.Min(minHeightToNeightbor, (int)(-maxTraversableStep - currentFloor));
                        continue;
                    }

                    HeightSpan neightborSpan = heightSpans[neighborIndex];

                    //Retrieve the data relative to the neightbor span (need also to take into account the area below the base span)
                    //Which is represented by the default value assigned below
                    int baseNeighborFloor = (int)-maxTraversableStep;
                    int baseNeighborCeiling = (int)(neightborSpan.yMin * cellHeight);

                    if ((Mathf.Min(currentCeiling, baseNeighborCeiling) - currentFloor) > minTraversableHeight)
                    {
                        minHeightToNeightbor = Mathf.Min(minHeightToNeightbor, baseNeighborFloor - currentFloor);
                    }

                    do
                    {
                        baseNeighborFloor = (int)(neightborSpan.yMax * cellHeight);
                        baseNeighborCeiling = (neightborSpan.nextSpan != null) ? (int)(neightborSpan.nextSpan.yMin * cellHeight) : int.MaxValue;

                        if (Mathf.Min(currentCeiling, baseNeighborCeiling) - Mathf.Max(currentFloor, baseNeighborFloor) > minTraversableHeight)
                        {
                            minHeightToNeightbor = Mathf.Min(minHeightToNeightbor, baseNeighborFloor - currentFloor);
                        }

                        neightborSpan = neightborSpan.nextSpan;
                    } 
                    while (neightborSpan != null);
                }

                if (minHeightToNeightbor < -maxTraversableStep)
                {
                    currentSpan.walkable = false;
                }

                currentSpan = currentSpan.nextSpan;
            } 
        }
    }

#endregion
    
#region OpenHeightFieldGeneration

    /*
    To create the open height field multiple steps have to be performed:
    1. create all of the open spans. This is not that difficult, since we already have the solid height field, so we just have to "invert" it
    2. 
    */
    void CreateOpenHeightField()
    {
        CreateOpenSpans();

        //in order to properly generate regions, we need to know which open spans are connected to each other
        GenerateNeighborLinks();

        //to generate regions, we need to know which regions are "borders", basically are at the edge of the terrain
        FindBorderSpans();

        //once it is clear, which open span has which neighbors and which spans are "borders", 
        //a distance field can be generated, that represents the distance each open span has to a border
        GenerateDistanceField();

        GenerateRegions();
        
        HandleRegions();

        ReassignBorderSpan();
    }

    void CreateOpenSpans()
    {
        //iterate over all spans
        for (int i = 0; i < heightSpans.Length; i++)
        {
            var current = heightSpans[i];
            OpenSpan baseSpan = null;
            OpenSpan previousSpan = null;

            //go through all spans in the column
            while(current != null)
            {
                //ignore, if not walkable
                if(!current.walkable)
                {
                    current = current.nextSpan;
                    continue;
                }

                //determine the space between this span and the span above
                int floor = current.yMax;
                int ceiling = (current.nextSpan != null) ? current.nextSpan.yMin : int.MaxValue;

                //ignore, if space is too small to traverse
                if((ceiling - floor) * cellHeight < minTraversableHeight)
                {
                    current = current.nextSpan;
                    continue;
                }

                //create a new open span reaching from the current max to the min of the next solid span
                OpenSpan newSpan = new OpenSpan(current.x, floor, ceiling, current.z);

                //if this is the first span in the column, it will be placed inside the open spans array
                if(baseSpan == null)
                    baseSpan = newSpan;

                if(previousSpan != null)
                    previousSpan.nextSpan = newSpan;

                previousSpan = newSpan;
                current = current.nextSpan;
            }

            if(baseSpan != null)
                openSpans[GetGridIndex(baseSpan.x, baseSpan.z)] = baseSpan;
        }
    }


/**
Loop through all columns in the openSpan grid

for each open span in existence, check if there are any neighbors that are on a similar height 
- heightDifference < maxTraversableStep 
- minTraversableHeight
*/
    void GenerateNeighborLinks()
    {
        for (int i = 0; i < openSpans.Length; i++)
        {
            var current = openSpans[i];
            
            while(current != null)
            {
                for (int n = 0; n < 4; n++)
                {
                    Vector2 neighborPos = new Vector2(current.x, current.z) + offsets[n];
                    int neighborIndex = GetGridIndex((int)neighborPos.x, (int)neighborPos.y); 

                    OpenSpan neighbor = openSpans[neighborIndex];
                    while(neighbor != null)
                    {
                        //ignore the neighbor if it is not reachable, since it is either too high or too low 
                        //compared to the current span
                        if(Mathf.Abs(neighbor.yMin - current.yMin) * cellHeight > maxTraversableStep)
                        {
                            neighbor = neighbor.nextSpan;
                            continue;
                        }

                        //get the highest floor and lowest ceiling
                        int maxFloor = Mathf.Max(current.yMin, neighbor.yMin);
                        int minCeiling = Mathf.Min(current.yMax, neighbor.yMax);

                        //if the gap is large enough for an agent to pass through, we found a valid neighbor
                        if((minCeiling - maxFloor) * cellHeight > minTraversableHeight)
                        {
                            current.neighbors[n] = neighbor;
                            break;
                        }
                        neighbor = neighbor.nextSpan;
                    }
                }
                
                current = current.nextSpan;
            }
        }
    }

    /*
    Loop through all the open spans and check if they are missing any of their 8 neighbors.
    If yes, then they are considered a border span and their distanceToBorder is initialized to 0
    Otherwise it is initialized to +infinity.
    */
    void FindBorderSpans()
    {
        for (int i = 0; i < openSpans.Length; i++)
        {
            var current = openSpans[i];
            while(current != null)
            {
                bool isBorder = false;
                
                for (int n = 0; n < 4; n++)
                {
                    OpenSpan neighbor = current.neighbors[n];
                    if(neighbor == null)
                    {
                        isBorder = true;
                        break;
                    }

                    OpenSpan diagonalSpan = neighbor.GetDiagonalNeighbor(n); //todo: get diagonal neighbor
                    if(diagonalSpan == null)
                    {
                        isBorder = true;
                        break;
                    }
                }

                current.distanceToBorder = isBorder ? 0 : int.MaxValue;

                current = current.nextSpan;
            }
        }
    }

    /*
    Loop through all spans at the current position in the openSpans array.
    For each of them, c

    */
    void GenerateDistanceField(int index, bool calculateMinMax)
    {
        OpenSpan current = openSpans[index];
        while(current != null)
        {
            int distance = current.distanceToBorder;

            //ignore, if it is a border span, so the distanceToBorder is 0. There is no need to calculate its distanceToBorder
            //since that has already happened in FindBorderSpans()
            if(distance == 0)
            {
                current = current.nextSpan;
                continue;
            }

            int minDistance = int.MaxValue;
            //loop through all neighbors and check their distance to the border. 
            //check which of them has the minimum distance to the border
            for (int n = 0; n < 4; n++)
            {
                OpenSpan neighbor = current.neighbors[n];
                if(neighbor == null)
                    continue;
                
                if(neighbor.distanceToBorder < minDistance)
                    minDistance = neighbor.distanceToBorder;

                OpenSpan diagonalNeighboor = neighbor.GetDiagonalNeighbor(n);
                if(diagonalNeighboor == null)
                    continue;
                
                if(diagonalNeighboor.distanceToBorder < minDistance)
                    minDistance = diagonalNeighboor.distanceToBorder;
                
            }

            //if any of the neighbor had a valid distanceToBorder (so not +infinity, which means it is not yet calculated)
            //set the distanceToBorder of this span to the minimum distanceToBorder of all of its neighbors + 2
            //maybe this can be done better, by using a more sofisticated distance metric, like the euclidean distance
            current.distanceToBorder = minDistance != int.MaxValue ? minDistance + 2 : current.distanceToBorder;

            //if this is already the second iteration, then also update the global minimum and maximum values for border distance
            if(calculateMinMax)
            {
                minBorderDistance = Mathf.Min(minBorderDistance, current.distanceToBorder);
                maxBorderDistance = Mathf.Max(maxBorderDistance, current.distanceToBorder);
            }

            current = current.nextSpan;
        }
    }

    /*
    Loop through all cells twice, once forward and once backwards. This works somehow? Maybe check later for proof
    */
    void GenerateDistanceField()
    {
        for (int i = 0; i < openSpans.Length; i++)
            GenerateDistanceField(i, false);
        
        for (int i = openSpans.Length - 1; i >= 0; i--)
            GenerateDistanceField(i, true);
    }

    /*
    */
    void GenerateRegions()
    {
        int minDist = traversableAreaBorderSize + minBorderDistance;
        int currentDistance = maxBorderDistance;

        int expandIterations = 4 + traversableAreaBorderSize + 2;

        //start from 1, since 0 is the id of "no region" / "null region"
        int nextRegionId = 1;

        List<OpenSpan> spansToFlood = new List<OpenSpan>();

        while(currentDistance > minDist)
        {
            //loop through all spans. If there are any spans, that have no regionId and their distance is 
            //larger or equal to the currentDistance, at them to the floodedSpans list.
            //since, the currentDistance is initialized to the maximumDistance found in all spans,
            //this means that at the beginning only the spans at the center of the largest connected area will be chosen
            //and as currentDistance gets lower, the spans closer to the borders will get chosen
            for (int i = 0; i < openSpans.Length; i++)
            {
                OpenSpan current = openSpans[i];
                while(current != null)
                {
                    if(current.regionId == 0 && current.distanceToBorder >= currentDistance)
                        spansToFlood.Add(current);

                    current = current.nextSpan;
                }
            }

            //if there already exist more than 1 region, expand the floodedSpans
            if(nextRegionId > 1)
                ExpandRegions(spansToFlood, currentDistance > 0 ? expandIterations : -1);
            
            //loop through each of the flooded spans
            foreach (var current in spansToFlood)
            {
                //ignore spans that already belong to a region
                if(current.regionId != 0)
                    continue;
                
                //flood the region, starting from the current span
                int fillTo = Mathf.Max(currentDistance - 2, minDist);
                FloodNewRegion(current, fillTo, ref nextRegionId);
            }

            //reduce the currentDistance by 2, but not below the minimum
            //should this not be minDist?
            currentDistance = Mathf.Max(currentDistance - 2, minBorderDistance);
        }

        spansToFlood.Clear();
        for (int i = 0; i < openSpans.Length; i++)
        {
            OpenSpan current = openSpans[i];
            while(current != null)
            {
                if(current.regionId == 0 && current.distanceToBorder >= minDist)
                    spansToFlood.Add(current);

                current = current.nextSpan;
            }
        }

        ExpandRegions(spansToFlood, minDist > 0 ? expandIterations * 8 : -1);

        regionCount = nextRegionId;
    }

    void ExpandRegions(List<OpenSpan> spansToFlood, int maxIterations)
    {
        int totalSpansToFlood = spansToFlood.Count;

        if(totalSpansToFlood == 0)
            return;
    
        int it = 0;
        while(true)
        {
            int skipped = 0;

            //go through all spans in the spansToFlood array, that do not have a region yet
            foreach(var current in spansToFlood)
            {
                if(current.regionId != 0)
                {
                    ++skipped;
                    continue;
                }

                int spanRegion = 0;
                int regionCenterDistance = int.MaxValue;

                //go through all neighbors of the current span
                for (int n = 0; n < 4; n++)
                {
                    OpenSpan neighbor = current.neighbors[n];
                    if(neighbor == null)
                        continue;

                    //if a neighbor has a region assigned to it
                    if(neighbor.regionId != 0)
                    {
                        if(neighbor.distanceToRegionCenter + 2 < regionCenterDistance)
                        {
                            int sameRegionCount = 0;
                            if(useConservativeExpansion)
                            {
                                for (int nn = 0; nn < 4; nn++)
                                {
                                    OpenSpan nNeighbor = neighbor.neighbors[nn];
                                    if(nNeighbor == null)
                                        continue;
                                    
                                    if(nNeighbor.regionId == neighbor.regionId)
                                        ++sameRegionCount;
                                }
                            }

                            if(!useConservativeExpansion || sameRegionCount > 1)
                            {
                                spanRegion = neighbor.regionId;
                                regionCenterDistance = neighbor.distanceToRegionCenter + 2;
                            }
                        }
                    }
                }

                if(spanRegion != 0)
                {
                    current.regionId = spanRegion;
                    current.distanceToRegionCenter = regionCenterDistance;
                }
                else
                {
                    ++skipped;
                }
            }

            //all spans were already processed or skipped
            if(skipped == totalSpansToFlood)
                break;

            if(maxIterations != -1)
            {
                ++it;
                if(it > maxIterations)
                    break;
            }
        }
    }   

    void FloodNewRegion(OpenSpan rootSpan, int fillToDistance, ref int regionId)
    {
        int regionSize = 0;
        Queue<OpenSpan> queue = new Queue<OpenSpan>();
        rootSpan.regionId = regionId;
        rootSpan.distanceToRegionCenter = 0;
        queue.Enqueue(rootSpan);

        //continue, while the queue still holds spans
        while(queue.Count > 0)
        {
            OpenSpan current = queue.Dequeue();
            
            bool isInRegionBorder = false;

            //loop through all 8 neighbors of the current span
            //if any of the neighbors have already a different region assigned to them
            //that means, that this cell is on the border of the current region
            for (int n = 0; n < 4; n++)
            {
                OpenSpan neighbor = current.neighbors[n];
                if(neighbor == null)
                    continue;

                if(neighbor.regionId != 0 && neighbor.regionId != regionId)
                {
                    isInRegionBorder = true;
                    break;
                }

                OpenSpan diagonalNeighbor = neighbor.GetDiagonalNeighbor(n);
                if(diagonalNeighbor == null)
                    continue;
                
                if(diagonalNeighbor.regionId != 0 && diagonalNeighbor.regionId != regionId)
                {
                    isInRegionBorder = true;
                    break;
                }
            }

            //is this span is on the border of the current region, invalidate its region id

            if(isInRegionBorder)
            {
                current.regionId = 0;
                continue;
            }

            //if this point is reached, it means that the current span is part of the region
            ++regionSize;

            //now, look again at all of the neighbors of the current cell. 
            //if there exists some, which do not have a region yet and are close enough to the current  fillDistance,
            //add them to the current region and enqueue them.
            for (int n = 0; n < 4; n++)
            {
                OpenSpan neighbor = current.neighbors[n];
                if(neighbor == null)
                    continue;

                if(neighbor.distanceToBorder >= fillToDistance && neighbor.regionId == 0)
                {
                    neighbor.regionId = regionId;

                    //why this
                    neighbor.distanceToRegionCenter = 0;
                    queue.Enqueue(neighbor);
                }
            }
        }

        //if any cells are in this region, increment the region id
        if(regionSize > 0)
            ++regionId;
    }

    //check if params are in range
    void SetMinRegionParameters()
    {
        minUnconnectedRegionSize = Mathf.Max(1, minUnconnectedRegionSize);
        minMergeRegionSize = Mathf.Max(0, minMergeRegionSize);
    }
    void HandleRegions()
    {
        SetMinRegionParameters();
        if(regionCount < 2)
            return;

        regions = new Region[regionCount];
        for (int i = 0; i < regionCount; i++)
            regions[i] = new Region(i);
        
        GatherRegionData();
        RemoveSmallUnconnectedRegions();
        MergeRegions();
        RemapRegionAndSpanIds();
    }

    void GatherRegionData()
    {
        //iterate through all spans
        for (int i = 0; i < openSpans.Length; i++)
        {
            OpenSpan current = openSpans[i];
            while(current != null)
            {
                //only consider spans that have a region assigned to them
                //Are there any that dont?
                if(current.regionId == 0)
                {
                    current = current.nextSpan;
                    continue;
                }

                //increase the span count of the region this span belongs to
                Region region = regions[current.regionId];
                region.spanCount++;

                OpenSpan next = current.nextSpan;

                //go through all spans above the current span
                while(next != null)
                {
                    //only consider them, when they are part of a region
                    if(next.regionId == 0)
                    {
                        next = next.nextSpan;
                        continue;
                    }

                    //add them to this regions overlapping regions
                    if(!region.overlappingRegions.Contains(next.regionId))
                        region.overlappingRegions.Add(next.regionId);

                    next = next.nextSpan;
                }

                //only continue further, if the connections for this region have not been found yet
                if(region.connections.Count > 0)
                {
                    current = current.nextSpan;
                    continue;
                }

                int edgeDirection = current.GetRegionEdgeDirection();
                if(edgeDirection != -1)
                {
                    FindRegionConnections(current, edgeDirection, region.connections);
                }

                current = current.nextSpan;
            }
        }
    }

    void FindRegionConnections(OpenSpan span, int edgeDirection, List<int> regionConnections)
    {
        OpenSpan current = span;
        int lastEdgeRegionId = 0;
        int direction = edgeDirection;

        OpenSpan neighbor = current.neighbors[direction];
        if(neighbor != null)
            lastEdgeRegionId = neighbor.regionId;

        regionConnections.Add(lastEdgeRegionId);

        
        for (int i = 0; i < 100000; i++)
        {
            neighbor = current.neighbors[direction];
            int currentEdgeRegionId = 0;

            if(neighbor == null || neighbor.regionId != current.regionId)
            {
                if(neighbor != null)
                    currentEdgeRegionId = neighbor.regionId;

                if(currentEdgeRegionId != lastEdgeRegionId)
                {
                    regionConnections.Add(currentEdgeRegionId);
                    lastEdgeRegionId = currentEdgeRegionId;
                }

                direction = (direction + 1) % 4;
            }
            else
            {
                current = neighbor;
                direction = direction - 1;
                if(direction < 0)
                    direction = 3;
            }

            if(span == current && edgeDirection == direction)
                break;
        }

        //remove last element
        if(regionConnections.Count > 1 && regionConnections[0] == regionConnections[regionConnections.Count - 1])
            regionConnections.RemoveAt(regionConnections.Count - 1);   
    }

    void RemoveSmallUnconnectedRegions()
    {
        foreach (var region in regions)
        {
            if(region.spanCount == 0)
                continue;

            if(region.connections.Count == 1 && region.connections[0] == 0)
            {
                if(region.spanCount < minUnconnectedRegionSize)
                    region.Reset(0);
            }
        }
    }

    void MergeRegions()
    {
        int mergeCount;

        do
        {
            mergeCount = 0;

            //skip irrelevant regions for merging
            foreach (var region in regions)
            {
                if(region.id == 0 || region.spanCount == 0 || region.spanCount > minMergeRegionSize)
                    continue;

                Region targetMergeRegion = null;
                int smallestSizeFound = int.MaxValue;

                foreach (var regionId in region.connections)
                {
                    if(regionId == 0 || regionId == region.id)
                        continue;

                    Region other = regions[regionId];
                    if(other.spanCount < smallestSizeFound && region.CanRegionBeMerged(other))
                    {
                        targetMergeRegion = other;
                        smallestSizeFound = other.spanCount;
                    }
                }

                if(targetMergeRegion != null && region.PerformRegionMergingInto(targetMergeRegion))
                {
                    int oldRegionId = region.id;
                    region.Reset(targetMergeRegion.id);

                    foreach(var other in regions)
                    {
                        if(other.id == 0)
                            continue;

                        if(other.id == oldRegionId)
                        {
                            other.id = targetMergeRegion.id;
                        }
                        else
                        {
                            other.ReplaceNeighborRegionId(oldRegionId, targetMergeRegion.id);
                        }
                    }
                    ++mergeCount;
                }
            }
        }
        while(mergeCount > 0);
    }

    void RemapRegionAndSpanIds()
    {
        foreach (var region in regions)
        {
            if(region.id > 0)
                region.idRemapNeeded = true;
        }

        int currentRegionId = 0;
        foreach (var region in regions)
        {
            if(!region.idRemapNeeded)
                continue;

            ++currentRegionId;
            int oldId = region.id;

            foreach (var other in regions)
            {
                if(other.id == oldId)
                {
                    other.id = currentRegionId;
                    other.idRemapNeeded = false;
                }
            }
        }

        regionCount = currentRegionId + 1;

        foreach (var openSpan in openSpans)
        {
            OpenSpan current = openSpan;
            while(current != null)
            {
                if(current.regionId == 0)
                {
                    current = current.nextSpan;
                    continue;
                }

                current.regionId = regions[current.regionId].id;
                current = current.nextSpan;
            }
        }
    }

    void ReassignBorderSpan()
    {
        bool spanChanged = true;

        while(spanChanged)
        {
            spanChanged = false;

            foreach (var span in openSpans)
            {
                OpenSpan current = span;
                while(current != null)
                {
                    if(current.regionId == 0)
                    {
                        current = current.nextSpan;
                        continue;
                    }

                    for (int n = 0; n < 4; n++)
                    {
                        OpenSpan neighbor = current.neighbors[n];
                        int nInc = (n + 1) % 4;
                        int nDec = n - 1;
                        if(nDec < 0)
                            nDec = 3;

                        OpenSpan neighborInc = current.neighbors[nInc];
                        OpenSpan neighborDec = current.neighbors[nDec];

                        int nIncId = neighborInc != null ? neighborInc.regionId : 0;
                        int nDecId = neighborDec != null ? neighborDec.regionId : 0;

                        if(current.regionId != neighbor.regionId && neighbor.regionId != 0 && 
                        (neighbor.regionId == nIncId || neighbor.regionId == nDecId))
                        {
                            current.regionId = neighbor.regionId;
                            spanChanged = true;
                            break;
                        }
                    }

                    current = current.nextSpan;
                }
            }
        }
    }
#endregion

#region ContourGeneration
    void GenerateContour()
    {
        int discardedContour = 0;

        FindNeighborRegionConnection(ref discardedContour);

        foreach (var openSpan in openSpans)
        {
            OpenSpan current = openSpan;

            while(current != null)
            {
                if(current.regionId == 0 || !current.AnyNeighborRegionFlag())
                {
                    current = current.nextSpan;
                    continue;
                }

                List<ContourVertexData> rawVertices = new List<ContourVertexData>();
                List<ContourVertexData> currentSimplifiedVertices = new List<ContourVertexData>();

                int neighborDirection = current.GetFirstNeighborFlag();
                bool onlyNullRegionConnected = true;

                BuildRawContours(current, neighborDirection, ref onlyNullRegionConnected, rawVertices);
                BuildSimplifiedContour(onlyNullRegionConnected, rawVertices, currentSimplifiedVertices);

                foreach (var vertex in currentSimplifiedVertices)
                    simplifiedVertices.Add(vertex);
                
                current = current.nextSpan;
            }
        }
    }

    void FindNeighborRegionConnection(ref int numberOfDiscardedContours)
    {
        foreach(var span in openSpans)
        {
            OpenSpan current = span;
            while(current != null)
            {
                if(current.regionId == 0)
                {
                    current = current.nextSpan;
                    continue;
                }

                int diffNeighborId = 0;
                for (int n = 0; n < 4; n++)
                {
                    int neighborId = 0;
                    OpenSpan neighbor = current.neighbors[n];

                    if(neighbor != null)
                        neighborId = neighbor.regionId;

                    if(current.regionId != neighborId)
                    {
                        current.neighborInDifferentRegions[n] = true;
                        ++diffNeighborId;
                    }
                }

                //If all the neighbors are part of a different region, the span considered is an island span
                //No need to process it for the contour generation
                if(diffNeighborId == 4)
                {
                    current.ResetNeighborRegionFlags();
                    ++numberOfDiscardedContours;
                }

                current = current.nextSpan;
            }
        }
    }

    void BuildRawContours(OpenSpan span, int startDirection, ref bool onlyNullRegionConnected, List<ContourVertexData> rawVertices)
    {
        int indexRaw = 0;

        OpenSpan current = span;
        int direction = startDirection;
        int startX = current.x;
        int startZ = current.z;

        for (int i = 0; i < 100000; i++)
        {
            if(current.neighborInDifferentRegions[direction])
            {
                float x = navMeshBounds.min.x + cellSize * startX;
                float y = navMeshBounds.min.y + cellHeight * GetCornerYIndex(current, direction);
                float z = navMeshBounds.min.z + cellSize * startZ + cellSize; //why +cellsize?
            
                switch(direction)
                {
                    case 0: z -= cellSize; break;
                    case 1: x += cellSize; z -= cellSize; break;
                    case 2: x += cellSize; break;
                }

                int regionIdDirection = 0;
                OpenSpan neighborSpan = current.neighbors[direction];
                if(neighborSpan != null)
                    regionIdDirection = neighborSpan.regionId;
                
                if(regionIdDirection != 0 && onlyNullRegionConnected)
                    onlyNullRegionConnected = false;

                rawVertices.Add(new ContourVertexData(new Vector3(x,y,z), regionIdDirection, current.regionId, indexRaw));

                current.neighborInDifferentRegions[direction] = false;
                direction = (direction + 1) % 4;
                ++indexRaw;
            }
            else
            {
                current = current.neighbors[direction];
                switch(direction)
                {
                    case 0: --startX; break;
                    case 1: --startZ; break;
                    case 2: ++startX; break;
                    case 3: ++startZ; break;
                }

                direction = (direction + 3) % 4;
            }

            if(current != null && direction == startDirection)
                break;
        }
    }

    int GetCornerYIndex(OpenSpan span, int neighborDirection)
    {
        int maxFloor = span.yMin;

        OpenSpan diagonalSpan = null;

        int directionOffset = (neighborDirection + 1) % 4;

        OpenSpan neighborSpan = span.neighbors[neighborDirection];
        if(neighborSpan != null)
        {
            maxFloor = Mathf.Max(maxFloor, neighborSpan.yMin);
            diagonalSpan = neighborSpan.neighbors[directionOffset];
        }

        neighborSpan = span.neighbors[directionOffset];
        if(neighborSpan != null)
        {
            maxFloor = Mathf.Max(maxFloor, neighborSpan.yMin);
            if(diagonalSpan == null)
                diagonalSpan = neighborSpan.neighbors[neighborDirection];
        }

        if(diagonalSpan != null)
            maxFloor = Mathf.Max(maxFloor, diagonalSpan.yMin);

        return maxFloor;
    }

    void BuildSimplifiedContour(bool onlyNullRegionConnected, List<ContourVertexData> rawVertices, List<ContourVertexData> currentSimplifiedVertices)
    {
        if(rawVertices.Count == 0)
            return;

        if(onlyNullRegionConnected)
        {
            ContourVertexData bottomLeft = rawVertices[0];
            ContourVertexData topRight = rawVertices[0];

            foreach(var vertex in rawVertices)
            {
                Vector3 position = vertex.position;

                if(position.x < bottomLeft.position.x || (position.x == bottomLeft.position.x && position.z < bottomLeft.position.z))
                    bottomLeft = vertex;

                if(position.x >= topRight.position.x || (position.x == topRight.position.x && position.z > topRight.position.z))
                    topRight = vertex;
            }

            currentSimplifiedVertices.Add(bottomLeft);
            currentSimplifiedVertices.Add(topRight);
        }
        else
        {
            for (int i = 0; i < rawVertices.Count; i++)
            {
                int region1 = rawVertices[i].externalRegionId;
                int nextIndex = (i + 1) % rawVertices.Count;
                int region2 = rawVertices[nextIndex].externalRegionId;

                if(region1 != region2)
                    currentSimplifiedVertices.Add(rawVertices[i]);
            }
        }

        if(currentSimplifiedVertices.Count == 0)
            return;

        ReinsertNullRegionVertices(rawVertices, currentSimplifiedVertices);
        CheckNullRegionMaxEdge(rawVertices, currentSimplifiedVertices);
        RemoveDuplicateVertices(currentSimplifiedVertices);
    }

    void ReinsertNullRegionVertices(List<ContourVertexData> rawVertices, List<ContourVertexData> currentSimplifiedVertices)
    {
        int rawCount = rawVertices.Count;
        int simplifiedCount = currentSimplifiedVertices.Count;

        int vertexA = 0;
        while(vertexA < simplifiedCount)
        {
            int vertexB = (vertexA + 1) % simplifiedCount;
            int rawIndexA = currentSimplifiedVertices[vertexA].rawIndex;
            int rawIndexB = currentSimplifiedVertices[vertexB].rawIndex;
            int vertexToTest = (rawIndexA + 1) % rawCount;

            float maximumDeviation = 0.0f;
            int vertexToInsert = -1;

            if(rawVertices[vertexToTest % rawCount].externalRegionId == 0)
            {
                while(vertexToTest != rawIndexB)
                {
                    float deviation = Math.PointDistanceToSegment(
                        rawVertices[vertexToTest].position, 
                        currentSimplifiedVertices[vertexA].position, 
                        currentSimplifiedVertices[vertexB].position);

                    if(deviation > maximumDeviation)
                    {
                        maximumDeviation = deviation;
                        vertexToInsert = vertexToTest;
                    }

                    vertexToTest = (vertexToTest + 1) % rawCount;
                }
            }

            if(vertexToInsert != -1 && maximumDeviation > edgeMaxDeviation)
            {
                currentSimplifiedVertices.Insert(vertexA + 1, rawVertices[vertexToInsert]);
                ++simplifiedCount;
            }
            else
            {
                ++vertexA;
            }
        }
    }

    void CheckNullRegionMaxEdge(List<ContourVertexData> rawVertices, List<ContourVertexData> currentSimplifiedVertices)
    {
        if(maxEdgeLength < cellSize)
            return;
    
        int rawCount = rawVertices.Count;
        int simplifiedCount = currentSimplifiedVertices.Count;

        int vertexA = 0;
        int it = 0;
        int maxIt = 100000;
        while(vertexA < simplifiedCount && it < maxIt)
        {
            it++;
            int vertexB = (vertexA + 1) % simplifiedCount;
            int rawIndexA = currentSimplifiedVertices[vertexA].rawIndex;
            int rawIndexB = currentSimplifiedVertices[vertexB].rawIndex;

            int newVertex = -1;
            int vertexToTest = (rawIndexA + 1) % simplifiedCount;

            if(rawVertices[vertexToTest % rawCount].externalRegionId == 0)
            {
                Vector3 dist = rawVertices[rawIndexB].position - rawVertices[rawIndexA].position;
                if(dist.x * dist.x + dist.z * dist.z > maxEdgeLength * maxEdgeLength)
                {
                    int distance = rawIndexB < rawIndexA ? rawIndexB + (rawCount - rawIndexA) : rawIndexB - rawIndexA;
                    newVertex = (rawIndexA + distance / 2) % rawCount;
                }
            }

            if(newVertex != -1)
            {
                currentSimplifiedVertices.Insert(vertexA + 1, rawVertices[newVertex]);
                ++simplifiedCount;
            }
            else
            {
                ++vertexA;
            }
        }
        if(it == maxIt)
            Debug.Log("Maxit reached");
    }

    void RemoveDuplicateVertices(List<ContourVertexData> currentSimplifiedVertices)
    {
        int count = currentSimplifiedVertices.Count;

        for (int i = 0; i < count; i++)
        {
            int next = (i + 1) % count;
            if(currentSimplifiedVertices[i].position == currentSimplifiedVertices[next].position)
            {
                currentSimplifiedVertices.RemoveAt(next);
                --count;
            }
        }
    }

#endregion

#region POLYGON

    Mesh Triangulate(List<Vector3> polygon)
    {
        if(IsConvex(polygon))
        {
            return TriangulateConvex(polygon);
        }
        else
        {
            var convexPieces = ConvexDecompose(polygon);
            Mesh[] triangleMeshes = new Mesh[convexPieces.Count];

            for (int i = 0; i < triangleMeshes.Length; i++)
            {
                triangleMeshes[i] = TriangulateConvex(convexPieces[i]);
            }

            return ConnectConvexPieces(triangleMeshes);
        }
    }

    bool IsConvex(List<Vector3> polygon)
    {
        return false;
    }

    List<List<Vector3>> ConvexDecompose(List<Vector3> polygon)
    {
        return null;
    }

    Mesh TriangulateConvex(List<Vector3> polygon)
    {   
        return null;
    }

    Mesh ConnectConvexPieces(Mesh[] convexPieces)
    {
        return null;
    }




#endregion

#region DRAW
    void OnDrawGizmos() 
    {
        Vector3 cellCenterOffset = new Vector3(cellSize * 0.5f, 0.0f, cellSize * 0.5f);
        Color minDistanceToBorderColor = Color.white;// new Color(0.4f, 0.8f, 1.0f, 0.5f);
        Color maxDistanceToBorderColor = Color.black;// new Color(0.2f, 0.2f, 0.22f, 0.5f);
        
        Random.InitState(0);
        Color[] regionColors = new Color[regionCount];
        for (int i = 0; i < regionCount; i++)
        {
            regionColors[i] = new Color(Random.Range(0.3f,1.0f),Random.Range(0.3f,1.0f),Random.Range(0.3f,1.0f),1.0f);
        }
        
        if(drawRegions && openSpans != null)
        {
            regionColors[0] = Color.black;

            //Iterate through all the cells
            for (int z = 0, idx = 0; z < zSize; z++)
            {
                for (int x = 0; x < xSize; x++, idx++)
                {
            
                    OpenSpan span = openSpans[idx];
                    while(span != null)
                    {
                        Vector3 spanMinCoord = new Vector3(navMeshBounds.min.x + cellSize * x, navMeshBounds.min.y + cellHeight * span.yMin, navMeshBounds.min.z + cellSize * z);
                        Vector3 centerA = spanMinCoord + cellCenterOffset;
                        Vector3 extents = new Vector3(cellSize * 0.9f, cellSize * 0.03f, cellSize * 0.9f);

                        Color c;
                        
                        if(drawRegions)
                        {
                            c = regionColors[span.regionId];                            
                        }
                        else //draw Distance Field
                        {
                            if(span.distanceToBorder == 0)
                            {
                                c = Color.black;// lightBlueishGrey;
                            }
                            else
                            {
                                float percentage = span.distanceToBorder / (float)(maxDistanceToBorder / 2.0f);
                                //Debug.Log(percentage);
                                c = Color.Lerp(minDistanceToBorderColor, maxDistanceToBorderColor, percentage);
                            } 
                        }
                        
                        Gizmos.color = c;
                        Gizmos.DrawCube(centerA, extents);
                        //draw links to neighbors
                        for (int n = 0; n < 4; n++)
                        {
                            OpenSpan neighbor = span.neighbors[n];
                            if(neighbor == null)
                                continue;
                            /*
                            Vector3 centerB = new Vector3(navMeshBounds.min.x + cellSize * neighbor.x, navMeshBounds.min.y + cellHeight * neighbor.yMin, navMeshBounds.min.z + cellSize * neighbor.z) + cellCenterOffset;
                            Vector3 pointA = centerA + (centerB - centerA) * 0.4f;
                            Vector3 pointB = centerB + (centerA - centerB) * 0.4f;

                            Gizmos.color = Color.magenta;
                            Gizmos.DrawLine(pointA, pointB);
                            */
                        }

                        if(drawDistanceNumbers)
                            DrawString(span.distanceToBorder.ToString(), centerA, Color.black);
                        
                        span = span.nextSpan;
                    }
                }
            }
        }
    
        if(drawRegionContours && simplifiedVertices != null)
        {
            List<Vector3> vertexBuffer = new List<Vector3>();
            for (int i = 1; i < regionCount; i++)
            {
                foreach (var vertex in simplifiedVertices)
                {
                    if(vertex.internalRegionId == i)
                        vertexBuffer.Add(vertex.position);
                }

                Gizmos.color = regionColors[i];
                if (vertexBuffer.Count < 3)
                    return;
                
                Vector3 secondPoint = vertexBuffer[vertexBuffer.Count - 1];

                foreach (var firstPoint in vertexBuffer)
                {
                    Gizmos.DrawSphere(firstPoint, 0.05f);
                    Gizmos.DrawLine(firstPoint, secondPoint);
                    secondPoint = firstPoint;
                }

                vertexBuffer.Clear();
            }

        }
    }

	static void DrawString(string text, Vector3 worldPos, Color? colour = null) {
		UnityEditor.Handles.BeginGUI();
		if (colour.HasValue) GUI.color = colour.Value;
		var view = UnityEditor.SceneView.currentDrawingSceneView;
		Vector3 screenPos = view.camera.WorldToScreenPoint(worldPos);
		Vector2 size = GUI.skin.label.CalcSize(new GUIContent(text));
		GUI.Label(new Rect(screenPos.x - (size.x / 2), -screenPos.y + view.position.height + 4, size.x, size.y), text);
		UnityEditor.Handles.EndGUI();
	}
    public void DrawGrid()
    {
        for (int x = 0; x < xSize; x++)
        {
            for (int y = 0; y < ySize; y++)
            {
                for (int z = 0; z < zSize; z++)
                {
                    Vector3 center = navMeshBounds.min + new Vector3(x,y,z) * cellSize + new Vector3(1,1,1) * cellSize * 0.9f;
                    Vector3 extent = new Vector3(cellSize, cellSize, cellSize) * 0.5f;
                    DrawBox(center, Quaternion.identity, extent, Color.grey, 4.0f);
                }
            }
        }
    }

    void DrawDebugHeightSpanData()
    {
        //Iterate through all the cells
        for (int x = 0; x < xSize; x++)
        {
            for (int z = 0; z < zSize; z++)
            {
                //Find the index of each span and make sure the index exist before accessing the map element
                int spanIndex = z * xSize + x;

                HeightSpan span = heightSpans[spanIndex];
                while (span != null)
                {
                    //Retrieve the location value of every span based on the bounds and the width and depth coordinates
                    Vector3 spanMinCoord = new Vector3(navMeshBounds.min.x + cellSize * x, navMeshBounds.min.y + cellHeight * span.yMin, navMeshBounds.min.z + cellSize * z);
                    Vector3 spanMaxCoord = new Vector3(spanMinCoord.x + cellSize, navMeshBounds.min.y + cellHeight * span.yMax, spanMinCoord.z + cellSize);
                    Color spanLineColor;

                    //Mark the spans with different colors based on their type
                    spanLineColor = span.walkable ? Color.green : Color.red;
                    
                    DrawMinMaxBox(spanMinCoord, spanMaxCoord, spanLineColor, debugDuration);

                    span = span.nextSpan;
                }
            }
        }
    }

    void DrawDebugOpenSpanData()
    {
        Vector3 cellCenterOffset = new Vector3(cellSize * 0.5f, 0.0f, cellSize * 0.5f);
        Color lightBlueishGrey = new Color(0.7f, 0.7f, 0.75f, 1.0f);
        //Iterate through all the cells
        for (int x = 0; x < xSize; x++)
        {
            for (int z = 0; z < zSize; z++)
            {
                //Find the index of each span and make sure the index exist before accessing the map element
                int spanIndex = z * xSize + x;

                OpenSpan span = openSpans[spanIndex];
                while(span != null)
                {
                    //Retrieve the location value of every span based on the bounds and the width and depth coordinates
                    Vector3 spanMinCoord = new Vector3(navMeshBounds.min.x + cellSize * x, navMeshBounds.min.y + cellHeight * span.yMin, navMeshBounds.min.z + cellSize * z);
                    //float yMax = span.yMax == int.MaxValue ? spanMinCoord.y + cellSize * 0.2f : navMeshBounds.min.y + cellHeight * span.yMax - cellHeight * 0.2f;
                    Vector3 spanMaxCoord = new Vector3(spanMinCoord.x + cellSize, spanMinCoord.y + cellSize * 0.2f, spanMinCoord.z + cellSize);

                    Color spanColor = span.distanceToBorder == 0 ? lightBlueishGrey : Color.cyan;
                    DrawMinMaxBox(spanMinCoord, spanMaxCoord, spanColor, debugDuration);
                    //draw links to neighbors
                    for (int n = 0; n < 4; n++)
                    {
                        OpenSpan neighbor = span.neighbors[n];
                        if(neighbor == null)
                            continue;

                        Vector3 centerA = spanMinCoord + cellCenterOffset;
                        Vector3 centerB = new Vector3(navMeshBounds.min.x + cellSize * neighbor.x, navMeshBounds.min.y + cellHeight * neighbor.yMin, navMeshBounds.min.z + cellSize * neighbor.z) + cellCenterOffset;
                        Vector3 pointA = centerA + (centerB - centerA) * 0.4f;
                        Vector3 pointB = centerB + (centerA - centerB) * 0.4f;

                        Debug.DrawLine(pointA, pointB, Color.magenta, debugDuration);
                    }
                    span = span.nextSpan;
                }
            }
        }
    }

    void DrawMeshFaces(Vector3[] data, Color color, float duration)
    {
		int ArrayLength = data.Length;
		if (ArrayLength % 3 != 0)
		{
            Debug.Log("Incorrect number of vertices passed in, the number must be divided by 3");
			return;
		}

        for (int i = 0; i < ArrayLength; i+=3)
        {
            Debug.DrawLine(data[i  ], data[i+1], color, duration);
            Debug.DrawLine(data[i+1], data[i+2], color, duration);
            Debug.DrawLine(data[i  ], data[i+2], color, duration);
        }
    }

    void DrawMinMaxBox(Vector3 min, Vector3 max, Color color, float duration)
    {
        Vector3 center = (max + min) * 0.5f;
		Vector3 extent = (max - min);// * 1.2f;
        DrawBox(center, Quaternion.identity, extent, color, duration);
    }


    public void DrawBox(Vector3 pos, Quaternion rot, Vector3 scale, Color c, float duration)
    {
            // create matrix
        Matrix4x4 m = new Matrix4x4();
        m.SetTRS(pos, rot, scale);

        var point1 = m.MultiplyPoint(new Vector3(-0.5f, -0.5f, 0.5f));
        var point2 = m.MultiplyPoint(new Vector3(0.5f, -0.5f, 0.5f));
        var point3 = m.MultiplyPoint(new Vector3(0.5f, -0.5f, -0.5f));
        var point4 = m.MultiplyPoint(new Vector3(-0.5f, -0.5f, -0.5f));

        var point5 = m.MultiplyPoint(new Vector3(-0.5f, 0.5f, 0.5f));
        var point6 = m.MultiplyPoint(new Vector3(0.5f, 0.5f, 0.5f));
        var point7 = m.MultiplyPoint(new Vector3(0.5f, 0.5f, -0.5f));
        var point8 = m.MultiplyPoint(new Vector3(-0.5f, 0.5f, -0.5f));

        Debug.DrawLine(point1, point2, c, duration);
        Debug.DrawLine(point2, point3, c, duration);
        Debug.DrawLine(point3, point4, c, duration);
        Debug.DrawLine(point4, point1, c, duration);

        Debug.DrawLine(point5, point6, c, duration);
        Debug.DrawLine(point6, point7, c, duration);
        Debug.DrawLine(point7, point8, c, duration);
        Debug.DrawLine(point8, point5, c, duration);

        Debug.DrawLine(point1, point5, c, duration);
        Debug.DrawLine(point2, point6, c, duration);
        Debug.DrawLine(point3, point7, c, duration);
        Debug.DrawLine(point4, point8, c, duration);
    }

    public void DrawPolygon(List<Vector3> vertices, Color color, float duration)
    {
        int arrayLength = vertices.Count;
		if (arrayLength < 3)
			return;
		
		Vector3 secondPoint = vertices[arrayLength - 1];

		foreach (var firstPoint in vertices)
		{
			Debug.DrawLine(firstPoint, secondPoint, color, duration);
			secondPoint = firstPoint;
		}
    }
#endregion DRAW
}