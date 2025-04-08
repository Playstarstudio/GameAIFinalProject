using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;

[CustomEditor(typeof(MapDisplay))]
public class DisplayTracker : Editor {
    public override void OnInspectorGUI()
    {
        MapDisplay mapDisplay = (MapDisplay)target;

        DrawDefaultInspector();

        // Button to apply changes and recalculate the map
        if (GUILayout.Button("Recalculate Map"))
        {
            // adjust terrain types and regenerate the map
            mapDisplay.OnSliderChange();
        }
    }
}
