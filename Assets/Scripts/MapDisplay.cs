using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Tilemaps;

public class MapDisplay : MonoBehaviour
{
    public Tilemap tilemap; 
    public Tile baseTile; 

    public void DrawNoiseMap(float[,] noiseMap)
    {
        int width = noiseMap.GetLength(0);
        int height = noiseMap.GetLength(1);

        // Clear old tiles before drawing
        tilemap.ClearAllTiles();
        // Ensure the Tilemap bounds match the noise map
        tilemap.ResizeBounds(); 
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                float value = noiseMap[x, y];
                Color tileColor = Color.Lerp(Color.black, Color.white, value);
                Tile newTile = ScriptableObject.CreateInstance<Tile>();
                newTile.sprite = baseTile.sprite; 
                newTile.color = tileColor; 

                int centeredX = x - width / 2;
                int centeredY = y - height / 2;
                tilemap.SetTile(new Vector3Int(centeredX, centeredY, 0), newTile);
            }
        }
    }
}
