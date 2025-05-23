using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(MapGenerator))]
public class GeneratorTracker : Editor
{
    public override void OnInspectorGUI()
    {
        MapGenerator mapGen = (MapGenerator)target;
        DrawDefaultInspector();
    }
}
