using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(MapGenerator))]
public class GeneratorTracker : Editor
{
    public override void OnInspectorGUI()
    {
        MapGenerator mapGen = (MapGenerator)target;

        if (DrawDefaultInspector())
        {
            if (mapGen.autoUpdate)
            {
                mapGen.GenerateMap();
            }
        }
    }
}
