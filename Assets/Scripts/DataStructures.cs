using System.Collections.Generic;
using UnityEngine;

/*
A span in the solid height field. Contains information about its coordinates, whether it is walkable and which span comes above it
*/
public class HeightSpan
{
    public HeightSpan(int x, int yMin, int yMax, int z, bool walkable)
    {
        this.x = x;
        this.yMin = yMin;
        this.yMax = yMax;
        this.z = z;
        this.walkable = walkable;
    }

    public int x, yMin, yMax, z;
    public bool walkable;
    public HeightSpan nextSpan;
}

public class OpenSpan
{
    public OpenSpan(int x, int yMin, int yMax, int z)
    {
        this.x = x;
        this.yMin = yMin;
        this.yMax = yMax;
        this.z = z;
    }

    public int x, yMin, yMax, z, distanceToBorder, distanceToRegionCenter, regionId;
    public bool processedForRegionFixing = false;

    public OpenSpan nextSpan = null;
    public OpenSpan[] neighbors = { null, null, null, null};
    public bool[] neighborInDifferentRegions = {false, false, false, false};

    public OpenSpan GetDiagonalNeighbor(int direction)
    {
        return neighbors[(direction + 1) % 4];
    }

    //get the ids of all 8 neighbors. 
    public int[] GetNeighborRegionIds()
    {
        int[] neighborIds = new int[8];
        
        for (int n = 0; n < 4; n++)
        {
            OpenSpan neighbor = neighbors[n];
            if(neighbor == null)
            {
                neighborIds[n] = 0;
                neighborIds[n + 4] = 0;
                continue;
            }

            neighborIds[n] = neighbor.regionId;
            OpenSpan diagonalNeighbor = neighbor.GetDiagonalNeighbor(n);
            neighborIds[n + 4] = diagonalNeighbor == null ? 0 : diagonalNeighbor.regionId;
        }

        return neighborIds;
    }

    public int SelectedRegionId(int borderDirection, int cornerDirection)
    {
        int[] neighborRegionIds = GetNeighborRegionIds();
        return 0;
    }

    public void PartialFloodRegion(int borderDirection, int newRegionId)
    {

    }

    bool ProcessNullRegion(int startDirection)
    {
        return false;
    }

    bool ProcessOuterCorner(int borderDirection)
    {
        return false;
    }

    //return the first neighbor direction, in which there is no neighbor, or a neighbor belonging to a different region
    //which indicates that this span is on the edge of a region
    public int GetRegionEdgeDirection()
    {
        for (int n = 0; n < 4; n++)
        {
            OpenSpan neighbor = neighbors[n];
            if(neighbor == null || neighbor.regionId != regionId)
                return n;
        }

        return -1;
    }

    //return the first neighbor direction, in which there is a neighbor, that does not belong to region 0
    public int GetNonNullEdgeDirection()
    {
        for (int n = 0; n < 4; n++)
        {
            OpenSpan neighbor = neighbors[n];
            if(neighbor != null && neighbor.regionId != 0)
                return n;
        }

        return -1;
    }

    //return the first neighbor direction, in which either there is no neighbor, or a neighbor belonging to region 0
    public int GetNullEdgeDirection()
    {
        for (int n = 0; n < 4; n++)
        {
            OpenSpan neighbor = neighbors[n];
            if(neighbor == null || neighbor.regionId == 0)
                return n;
        }

        return -1;
    }

    public void ResetNeighborRegionFlags()
    {
        for (int i = 0; i < 4; i++)
            neighborInDifferentRegions[i] = false;
        
    }

    public int GetFirstNeighborFlag()
    {
        for (int n = 0; n < 4; n++)
        {
            if(neighborInDifferentRegions[n])
                return n;
        }
        return 4;
    }

    public bool AnyNeighborRegionFlag()
    {
        foreach (var flag in neighborInDifferentRegions)
        {
            if(flag)
                return true;
        }

        return false;
    } 
}

public class Region
{
    public Region(int id)
    {
        this.id = id;
        connections = new List<int>();
        overlappingRegions = new List<int>();
    }
    public int id = 0;
    public int spanCount = 0;
    public bool idRemapNeeded = false;
    public List<int> connections;
    public List<int> overlappingRegions;

    public void Reset(int newId)
    {
        id = newId;
        spanCount = 0;
        connections.Clear();
        overlappingRegions.Clear();
    }

    public bool CanRegionBeMerged(Region other)
    {
        int validConnections = 0;
        foreach (var connection in connections)
        {
            if(connection == other.id)
                ++validConnections;
        }

        // If the regions compared are 
        // 1 - Connecting in more than one point or they do not connect
        // 2 - Overlapping vertically
        // They cannot be merged
	    return !(validConnections != 1 || overlappingRegions.Contains(other.id) || other.overlappingRegions.Contains(id));
    }

    public bool PerformRegionMergingInto(Region targetRegion)
    {
        int connectionPointToTarget = targetRegion.connections.IndexOf(id);
        if(connectionPointToTarget == -1)
            return false;

        int connectionPointToCurrent = connections.IndexOf(targetRegion.id);
        if(connectionPointToCurrent == -1)
            return false;
        
        List<int> targetConnections = targetRegion.connections;
        List<int> newTargetConnections = new List<int>();

        int connectionSize = targetConnections.Count;
        for (int i = 0; i < connectionSize - 1; i++)
        {
            int connectionToAdd = targetConnections[(connectionPointToTarget + 1 + i) % connectionSize];
            newTargetConnections.Add(connectionToAdd);
        }

        connectionSize = connections.Count;
        for (int i = 0; i < connectionSize - 1; i++)
        {
            int connectionToAdd = connections[(connectionPointToTarget + 1 + i) % connectionSize];
            newTargetConnections.Add(connectionToAdd);
        }

        targetRegion.connections = newTargetConnections;
        targetRegion.RemoveAdjacentDuplicateConnections();

        foreach (var overlappingId in overlappingRegions)
        {
            if(!targetRegion.overlappingRegions.Contains(overlappingId))
                targetRegion.overlappingRegions.Add(overlappingId);
        }

        targetRegion.spanCount += spanCount;
        return true;
    }

    public void RemoveAdjacentDuplicateConnections()
    {
        int connection = 0;

        while(connection < connections.Count && connections.Count > 1)
        {
            int nextConnection = (connection + 1) % connections.Count;

            if(connections[connection] == connections[nextConnection])
            {
                connections.RemoveAt(nextConnection);
            }
            else
            {
                ++connection;
            }
        }
    }

    public void ReplaceNeighborRegionId(int oldId, int newId)
    {
        bool connectionChanged = false;

        for (int i = 0; i < connections.Count; i++)
        {
            if(connections[i] == oldId)
            {
                connections[i] = newId;
                connectionChanged = true;
            }
        }

        for (int i = 0; i < overlappingRegions.Count; i++)
        {
            if(overlappingRegions[i] == oldId)
                overlappingRegions[i] = newId;
        }

        if(connectionChanged)
            RemoveAdjacentDuplicateConnections();
    }
}

class ContourVertexData
{
    public ContourVertexData(){}

    public ContourVertexData(Vector3 position, int externalRegionId, int internalRegionId, int rawIndex)
    {
        this.position = position;
        this.externalRegionId = externalRegionId;
        this.internalRegionId = internalRegionId;
        this.rawIndex = rawIndex;
    }

    public Vector3 position;
    public int externalRegionId = 0;
    public int internalRegionId = 0;
    public int rawIndex = 0;
}
