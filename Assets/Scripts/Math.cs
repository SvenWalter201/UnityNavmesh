using UnityEngine;

class Math
{
    public static Vector3 LinePlaneIntersection(Vector3 point1, Vector3 point2, Vector3 planeOrigin, Vector3 planeNormal)
    {
        return point1 + (point2 - point1) * (Vector3.Dot(planeOrigin - point1,planeNormal) / Vector3.Dot(point2 - point1, planeNormal));
    }

    public static Vector3 ClosestPointOnSegment(Vector3 point, Vector3 startPoint, Vector3 endPoint)
    {
        Vector3 segment = endPoint - startPoint;
        Vector3 vectorToPoint = point - startPoint;

        float dot1 = Vector3.Dot(vectorToPoint, segment);
        if(dot1 <= 0)
            return startPoint;

        float dot2 = Vector3.Dot(segment, segment);
        if(dot2 <= dot1)
            return endPoint;

        return startPoint + segment * (dot1 / dot2);
    }
    
    public static float PointDistanceToSegment(Vector3 point, Vector3 startPoint, Vector3 endPoint)
    {
        Vector3 closestPoint = ClosestPointOnSegment(point, startPoint, endPoint);
        return (point - closestPoint).sqrMagnitude;
    }
}