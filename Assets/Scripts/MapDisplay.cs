using System.Collections.Generic;
using System.Net;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Tilemaps;

public class MapDisplay : MonoBehaviour
{
    public enum DisplayMode { NoiseMap, TerrainMap }
    public DisplayMode displayMode;
    public Tilemap tilemap;
    public Tile baseTile;
    private TerrainType[] terrainTypes;
    [Range(0f, 1f)]
    public float waterPercent = 0.2f;
    [Range(0f, 1f)]
    public float grasslandPercent = 0.2f;
    [Range(0f, 1f)]
    public float forrestPercent = 0.2f;
    [Range(0f, 1f)]
    public float mountainPercent = 0.2f;
    [Range(0f, 1f)]
    public float snowPercent = 0.1f;
    public MapGenerator mapGenerator;
    public float[,] buffer;
    public MapData[,] noiseMapData;

    private void Start()
    {
        adjustTerrainTypes(waterPercent, grasslandPercent, forrestPercent, mountainPercent, snowPercent);
    }

    //function to adjust the terrain based on the percentage chosen
    private void adjustTerrainTypes(float waterPercent, float grasslandPercent, float forrestPercent, float mountainPercent, float snowPercent)
    {
        float total = waterPercent + grasslandPercent + forrestPercent + mountainPercent + snowPercent;
        if (Mathf.Abs(total - 1.0f) > 0.01f)
        {
            Debug.LogWarning("Totals for terrain don't equal 1, adjusting now");
            float normalizer = 1 / (waterPercent + grasslandPercent + forrestPercent + mountainPercent + snowPercent);
            waterPercent = waterPercent * normalizer;
            grasslandPercent = grasslandPercent * normalizer;
            forrestPercent = forrestPercent * normalizer;
            mountainPercent = mountainPercent * normalizer;
            snowPercent = snowPercent * normalizer;
        }

        float waterThreshold = waterPercent;
        float grassThreshold = waterThreshold + grasslandPercent;
        float forestThreshold = grassThreshold + forrestPercent;
        float mountainThreshold = forestThreshold + mountainPercent;
        float snowThreshold = 1.0f;

        terrainTypes = new TerrainType[]
        {
            //in order
            new TerrainType(new Color(0.3f, 0.6f, 0.9f), false, waterThreshold, 0),    // Water
            new TerrainType(new Color(0.3f, 0.7f, 0.3f), true, grassThreshold, 0),     // Grasslands
            new TerrainType(new Color(0.1f, 0.4f, 0.1f), true, forestThreshold, 0),    // Forest
            new TerrainType(new Color(0.5f, 0.4f, 0.3f), false, mountainThreshold, 0), // Mountain
            new TerrainType(Color.white, false, snowThreshold, 0),                     // Snow
            new TerrainType(Color.red, true, 0.1f, 1),                                 // City
            new TerrainType(Color.yellow, true, 0.1f, 2),                              // Town
            new TerrainType(Color.gray,true, 0.1f ,3)                                  // Road
        };

        //redraw
        mapGenerator.GenerateMap();
    }

    public void OnSliderChange()
    {
        adjustTerrainTypes(waterPercent, grasslandPercent, forrestPercent, mountainPercent, snowPercent);
    }

    [ContextMenu("Recalculate the Map")]
    public void RecalculateTheMap()
    {
        OnSliderChange();
    }

    public void DrawNoiseMap(float[,] noiseMap, float[,] noiseMap2, float[,] noiseMap3)
    {
        if (terrainTypes == null)
        {
            //this should just show whenever we try to set a value that will push values over 1
            //triggering an adjustment and a normalization
            Debug.LogWarning("Terrain types not set up, trying to set up now");
            adjustTerrainTypes(waterPercent, grasslandPercent, forrestPercent, mountainPercent, snowPercent);
        }

        int width = noiseMap.GetLength(0);
        int height = noiseMap.GetLength(1);
        noiseMapData = new MapData[width, height];
        // Clear old tiles before drawing
        tilemap.ClearAllTiles();
        // Ensure the Tilemap bounds match the noise map
        tilemap.ResizeBounds();
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                float value = ((noiseMap[x, y] + (noiseMap2[x, y] * .3f) + (noiseMap3[x, y] * .125f) * .8f));
                //potentially clamp this
                value = Mathf.Clamp01(value);
                noiseMapData[x, y] = new MapData(value, GetTerrainType(value), 0);
                //use the enum to switch between either the map with the biomes or the noise map
                Color tileColour = Color.black;
                if (displayMode == DisplayMode.NoiseMap)
                {
                    //linearly interpolate between black and white based on value
                    tileColour = Color.Lerp(Color.black, Color.white, value);
                }
                else if (displayMode == DisplayMode.TerrainMap)
                {
                    //otherwise get the matching colour from the terrain type array
                    TerrainType selectedTerrain = GetTerrainType(noiseMapData[x, y].FloatValue);
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

    public void DrawSettlements(List<Vector2Int> cities, List<Vector2Int> towns, HashSet<Vector2Int> roads = null)
    {
        // Draw cities with red color
        foreach (Vector2Int city in cities)
        {
            int centeredX = city.x - tilemap.size.x / 2;
            int centeredY = city.y - tilemap.size.y / 2;

            Tile cityTile = ScriptableObject.CreateInstance<Tile>();
            cityTile.sprite = baseTile.sprite;
            cityTile.color = Color.red; // City color
            tilemap.SetTile(new Vector3Int(centeredX, centeredY, 0), cityTile);
        }

        // Draw towns with yellow color
        foreach (Vector2Int town in towns)
        {
            int centeredX = town.x - tilemap.size.x / 2;
            int centeredY = town.y - tilemap.size.y / 2;

            Tile townTile = ScriptableObject.CreateInstance<Tile>();
            townTile.sprite = baseTile.sprite;
            townTile.color = Color.yellow; // Town color

            tilemap.SetTile(new Vector3Int(centeredX, centeredY, 0), townTile);
        }

        // Draw roads if provided
        if (roads != null)
        {
            foreach (Vector2Int road in roads)
            {
                int centeredX = road.x - tilemap.size.x / 2;
                int centeredY = road.y - tilemap.size.y / 2;

                Tile roadTile = ScriptableObject.CreateInstance<Tile>();
                roadTile.sprite = baseTile.sprite;
                roadTile.color = new Color(0.7f, 0.7f, 0.7f); // Gray for roads

                tilemap.SetTile(new Vector3Int(centeredX, centeredY, 0), roadTile);
            }
        }

        Debug.Log("Drew cities, towns, and roads on tilemap");

    }

    //store the biome colour, whether the tile is traversable and the range for that terrain
    public struct TerrainType
    {
        public Color colour;
        public float threshold;
        public bool isTraversable;
        public int type;

        public TerrainType(Color colour, bool isTraversable, float threshold, int type)
        {
            this.colour = colour;
            this.isTraversable = isTraversable;
            this.threshold = threshold;
            this.type = type;
        }
    }

    public struct MapData
    {
        public float FloatValue;
        public TerrainType Terrain;
        public int Structure;

        public MapData (float floatValue, TerrainType terrain, int structure)
        {
            this.FloatValue = floatValue;
            this.Terrain = terrain;
            this.Structure = structure;
        }
    }

    //function to loop through the terrain types based on the value passed in
    private TerrainType GetTerrainType(float value)
    {
        TerrainType selectedTerrain = terrainTypes[terrainTypes.Length - 1];
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

    private TerrainType ReadSavedTerrain(int x, int y)
    {
        return noiseMapData[x, y].Terrain;
    }

    public float GetNoiseValue(int x, int y)
    {
        if (noiseMapData == null)
        {
            Debug.LogError("Noise map hasn't been saved yet!");
            return 0f;
        }
        if (x < 0 || x >= noiseMapData.GetLength(0) ||
            y < 0 || y >= noiseMapData.GetLength(1))
        {
            Debug.LogError($"Requested coordinates ({x},{y}) are out of bounds!");
            return 0f;
        }
        return noiseMapData[x, y].FloatValue;
    }

    public TerrainType GetTerrainType(int x, int y)
    {
        if (noiseMapData == null)
        {
            Debug.LogError("Noise map hasn't been initialized yet!");
            return default;
        }

        if (x < 0 || x >= noiseMapData.GetLength(0) ||
            y < 0 || y >= noiseMapData.GetLength(1))
        {
            Debug.LogError($"Requested coordinates ({x},{y}) are out of bounds!");
            return default;
        }

        return noiseMapData[x, y].Terrain;
    }
}