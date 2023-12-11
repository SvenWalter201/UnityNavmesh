using UnityEngine;
using UnityEngine.Events;

public static class UnityExtensions{

/// <summary>
    /// Extension method to check if a layer is in a layermask
    /// </summary>
    /// <param name="mask"></param>
    /// <param name="layer"></param>
    /// <returns></returns>
    public static bool Contains(this LayerMask mask, int layer)
    {
        return mask == (mask | (1 << layer));
    }

    public static float Aggregate(this Vector3 v3)
    {
        return v3.x + v3.y + v3.z;
    }

    public static Vector3 Mul(this Vector3 v3, Vector3 other)
    {
        return new Vector3(v3.x * other.x, v3.y * other.y, v3.z * other.z);
    }
}
