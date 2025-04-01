using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Tilemaps;
using UnityEngine.UIElements;

public class MapDisplay : MonoBehaviour
{
    public enum DisplayMode {NoiseMap, TerrainMap}
    public DisplayMode displayMode;
    public Tilemap tilemap; 
    public Tile baseTile;
    private TerrainType[] terrainTypes;


    private void Awake()
    {
        SetUpTerrainTypes();
    }

    private void SetUpTerrainTypes()
    {
        terrainTypes = new TerrainType[]
        {
            //in order
            //water, grassland, forrest, mountain, snow
            new TerrainType(new Color(0.3f, 0.6f, 0.9f), false, 0.3f),
            new TerrainType(new Color(0.3f, 0.7f, 0.3f), true, 0.5f),
            new TerrainType(new Color(0.1f,0.4f,0.1f), true, 0.7f),
            new TerrainType(new Color(0.5f, 0.4f, 0.3f), false, 0.9f),
            new TerrainType(Color.white, false, 1.0f),
        };
    }

    public void DrawNoiseMap(float[,] noiseMap, float[,] noiseMap2, float[,] noiseMap3)
    {
        if (terrainTypes == null)
        {
            Debug.LogWarning("Terrain types not set up, trying to set up now");
            SetUpTerrainTypes();
        }

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
                float value = (noiseMap[x, y] + (noiseMap2[x,y] * .3f) + (noiseMap3[x,y] * .125f))*.8f;
                
                //use the enum to switch between either the map with the biomes or the noise map
                Color tileColour = Color.black;
                if (displayMode == DisplayMode.NoiseMap)
                {
                    //linearly interpolate between black and white based on value
                    tileColour = Color.Lerp(Color.black, Color.white, value);
                } else if (displayMode == DisplayMode.TerrainMap)
                {
                    //otherwise get the matching colour from the terrain type array
                    TerrainType selectedTerrain = GetTerrainType(value);
                    tileColour = selectedTerrain.colour;
                }
                
                Tile newTile = ScriptableObject.CreateInstance<Tile>();
                newTile.sprite = baseTile.sprite; 
                newTile.color = tileColour; 

                int centeredX = x - width / 2;
                int centeredY = y - height / 2;
                tilemap.SetTile(new Vector3Int(centeredX, centeredY, 0), newTile);
            }
        }
    }

    //store the biome colour, whether the tile is traversable and the range for that terrain
    public struct TerrainType
    {
        public Color colour;
        public float threshold;
        public bool isTraversable;

        public TerrainType(Color colour, bool isTraversable, float threshold)
        {
            this.colour = colour;
            this.isTraversable = isTraversable;
            this.threshold = threshold;
        }
    }

    //function to loop through the terrain types based on the value passed in
    private TerrainType GetTerrainType(float value)
    {
        TerrainType selectedTerrain = new TerrainType();
        for (int i = 0; i < terrainTypes.Length; i++)
        {
            TerrainType myTerrain = terrainTypes[i];
            if (value < myTerrain.threshold)
            {
                selectedTerrain = myTerrain;
                break;
                //just stop once the terrain matching that range is found
            }
        }
        return selectedTerrain;
    }
}
