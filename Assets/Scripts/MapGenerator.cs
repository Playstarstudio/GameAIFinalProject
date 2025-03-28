using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MapGenerator : MonoBehaviour
{
    public int mapWidth = 300;
    public int mapHeight = 300;
    public float scale = 7.0f;

    public MapDisplay mapDisplay;

    void Start()
    {
        GenerateMap();
    }

    public void GenerateMap()
    {
        float[,] noiseMap = GenerateNoiseMap(mapWidth, mapHeight, scale);
        mapDisplay.DrawNoiseMap(noiseMap);
    }

    float[,] GenerateNoiseMap(int width, int height, float scale)
    {
        float[,] noiseMap = new float[width, height];
        //protecting against division by zero
        if (scale <= 0.0f)
        {
            scale = 0.001f;
        }

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                float sampleX = x / scale;
                float sampleY = y / scale;

                noiseMap[x, y] = Mathf.PerlinNoise(sampleX, sampleY);
            }
        }

        return noiseMap;
    }
}
