using UnityEngine;

public class MapGenerator : MonoBehaviour
{
    public int mapWidth;
    public int mapHeight;
    public float scale1;
    public float scale2;
    public float scale3;
    public int octaves1 = 1;
    public int octaves2 = 6;
    public int octaves3 = 12;
    [Range(0, 1)]
    public float persistence;
    public float lacunarity;
    public int seed;
    public Vector2 offset;
    public bool autoUpdate;
    public MapDisplay mapDisplay;

    void Start()
    {
        GenerateMap();
    }

    public void GenerateMap()
    {
        float[,] noiseMap1 = GenerateNoiseMap(mapWidth, mapHeight, seed, scale1, octaves1, persistence, lacunarity, offset);
        float[,] noiseMap2 = GenerateNoiseMap(mapWidth, mapHeight, seed, scale2, octaves2, persistence, lacunarity, offset);
        float[,] noiseMap3 = GenerateNoiseMap(mapWidth, mapHeight, seed, scale3, octaves3, persistence, lacunarity, offset);

        mapDisplay.DrawNoiseMap(noiseMap1, noiseMap2, noiseMap3);
    }

    float[,] GenerateNoiseMap(int width, int height, int seed, float scale, int octaves, float persistence, float lacunarity, Vector2 offset)
    {
        float[,] noiseMap = new float[width, height];
        //protecting against division by zero
        System.Random prng = new System.Random(seed);
        Vector2[] octaveOffsets = new Vector2[octaves];
        for (int  i = 0;  i < octaves;  i++)
        {
            float offsetX = prng.Next(-100000, 100000) + offset.x;
            float offsetY = prng.Next(-100000, 100000) + offset.y;
            octaveOffsets[i] = new Vector2(offsetX, offsetY);
        }
        if (scale <= 0.0f)
        {
            scale = 0.001f;
        }
        float maxNoiseHeight = float.MinValue;
        float minNoiseHeight = float.MaxValue;

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                float amplitude = 1;
                float frequency = 1;
                float noiseHeight = 0;
                for (int i = 0; i < octaves; i++)
                {
                    float sampleX = x / scale * frequency + octaveOffsets[i].x;
                    float sampleY = y / scale * frequency + octaveOffsets[i].y;
                    float perlinValue = Mathf.PerlinNoise(sampleX, sampleY) * 2 - 1;
                    noiseHeight += perlinValue * amplitude;
                    amplitude *= persistence;
                    frequency *= lacunarity;

                }
                if(noiseHeight > maxNoiseHeight)
                {
                    maxNoiseHeight = noiseHeight;
                }
                else if (noiseHeight < minNoiseHeight)
                {
                    minNoiseHeight = noiseHeight;
                }
                noiseMap[x, y] = noiseHeight;
            }
        }
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                noiseMap[x, y] = Mathf.InverseLerp(minNoiseHeight, maxNoiseHeight, noiseMap[x, y]);
            }
        }
        return noiseMap;
    }
}
