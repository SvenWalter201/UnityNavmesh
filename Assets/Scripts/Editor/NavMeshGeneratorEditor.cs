using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(NavMeshGenerator))]
public class NavMeshGeneratorEditor : Editor 
{
    NavMeshGenerator tgt;

    private void OnSceneGUI() 
    {
        
    }

    void Draw()
    {
        
    }

    public override void OnInspectorGUI() 
    {
        base.OnInspectorGUI();
        if(GUILayout.Button("DrawGrid"))
        {
            tgt.DrawGrid();
        }

        if (GUILayout.Button("Rebuild"))
        {
            tgt.Rebuild();        
        }
    }

    void OnEnable() 
    {
        tgt = target as NavMeshGenerator;
    }
}