using UnityEngine;
using System.Collections.Generic;

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

    [Header("Settlement Parameters")]
    public int numberOfCities = 5;
    public int numberOfTowns = 10;
    public int minCityDistance = 15; // Minimum distance between cities
    public int minTownDistance = 8;  // Minimum distance between towns
    public int roadMaxDistance = 10; // Maximum road distance from towns

    // Lists to track settlements
    private List<Vector2Int> cities = new List<Vector2Int>();
    private List<Vector2Int> towns = new List<Vector2Int>();

    [Header("Road Parameters")]
    public float roadCostMultiplier = 1.5f; // How much more expensive it is to go through difficult terrain

    // To store our road network
    private HashSet<Vector2Int> roadTiles = new HashSet<Vector2Int>();

    void Start()
    {
        GenerateMap();
    }

    public void GenerateMap()
    {
        // Generate noise maps as before
        float[,] noiseMap1 = GenerateNoiseMap(mapWidth, mapHeight, seed, scale1, octaves1, persistence, lacunarity, offset);
        float[,] noiseMap2 = GenerateNoiseMap(mapWidth, mapHeight, seed, scale2, octaves2, persistence, lacunarity, offset);
        float[,] noiseMap3 = GenerateNoiseMap(mapWidth, mapHeight, seed, scale3, octaves3, persistence, lacunarity, offset);

        // Draw terrain first
        mapDisplay.DrawNoiseMap(noiseMap1, noiseMap2, noiseMap3);

        // Place cities and towns
        PlaceSettlements();

        // Generate road network
        GenerateRoads();

        // Draw cities, towns, and roads on the tilemap
        mapDisplay.DrawSettlements(cities, towns, roadTiles);

        // Debug visualization
        VisualizeSettlements();
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

    private void PlaceSettlements()
    {
        cities.Clear();
        towns.Clear();

        // Place cities on suitable terrain
        PlaceCities();

        // Now also place towns
        PlaceTowns();
    }

    private void PlaceCities()
    {
        int attempts = 0;
        int maxAttempts = 1000;

        while (cities.Count < numberOfCities && attempts < maxAttempts)
        {
            attempts++;

            // Random position
            int x = Random.Range(0, mapWidth);
            int y = Random.Range(0, mapHeight);
            Vector2Int pos = new Vector2Int(x, y);

            // Check if position is on suitable terrain
            if (IsSuitableForCity(x, y))
            {
                // Check minimum distance from other cities
                bool tooClose = false;
                foreach (Vector2Int city in cities)
                {
                    if (Vector2Int.Distance(pos, city) < minCityDistance)
                    {
                        tooClose = true;
                        break;
                    }
                }

                if (!tooClose)
                {
                    cities.Add(pos);
                }
            }
        }

        Debug.Log($"Placed {cities.Count} cities after {attempts} attempts");
    }

    private bool IsSuitableForCity(int x, int y)
    {
        // For now, I am setting the return down as true. However, we'll improve this later in accordance with the terrain type
        // This will place cities anywhere, but we will need to refine it to only place on traversable terrain
        return true;
    }

    private void VisualizeSettlements()
    {
        // Create a temporary texture to visualize positions
        Texture2D texture = new Texture2D(mapWidth, mapHeight);

        // Set all pixels to transparent
        Color[] colorMap = new Color[mapWidth * mapHeight];
        for (int i = 0; i < colorMap.Length; i++)
        {
            colorMap[i] = Color.clear;
        }

        // Mark city positions with red
        foreach (Vector2Int city in cities)
        {
            int index = city.y * mapWidth + city.x;
            if (index >= 0 && index < colorMap.Length)
            {
                colorMap[index] = Color.red;
            }
        }

        // Mark town positions with yellow
        foreach (Vector2Int town in towns)
        {
            int index = town.y * mapWidth + town.x;
            if (index >= 0 && index < colorMap.Length)
            {
                colorMap[index] = Color.yellow;
            }
        }

        texture.SetPixels(colorMap);
        texture.Apply();

        Debug.Log("Settlements visualized!");
    }

    private void PlaceTowns()
    {
        int attempts = 0;
        int maxAttempts = 1000;

        while (towns.Count < numberOfTowns && attempts < maxAttempts)
        {
            attempts++;

            // Random position
            int x = Random.Range(0, mapWidth);
            int y = Random.Range(0, mapHeight);
            Vector2Int pos = new Vector2Int(x, y);

            // Check if position is on suitable terrain
            if (IsSuitableForTown(x, y))
            {
                // Check minimum distance from cities
                bool tooClose = false;

                // Check distance from cities
                foreach (Vector2Int city in cities)
                {
                    if (Vector2Int.Distance(pos, city) < minTownDistance)
                    {
                        tooClose = true;
                        break;
                    }
                }

                // Check distance from other towns
                if (!tooClose)
                {
                    foreach (Vector2Int town in towns)
                    {
                        if (Vector2Int.Distance(pos, town) < minTownDistance)
                        {
                            tooClose = true;
                            break;
                        }
                    }
                }

                if (!tooClose)
                {
                    towns.Add(pos);
                }
            }
        }

        Debug.Log($"Placed {towns.Count} towns after {attempts} attempts");
    }

    private bool IsSuitableForTown(int x, int y)
    {
        // For now, just return true like with cities
        // Later we'll improve this to check terrain type
        return true;
    }

    private void GenerateRoads()
    {
        roadTiles.Clear();

        // First connect all cities
        ConnectAllCities();

        // Then connect towns to nearest city
        ConnectTownsToNearestCity();

        Debug.Log($"Generated {roadTiles.Count} road tiles");
    }

    private void ConnectAllCities()
    {
        // If we have fewer than 2 cities, nothing to connect
        if (cities.Count < 2)
            return;

        // Simple approach: connect each city to its nearest unconnected city
        HashSet<Vector2Int> connectedCities = new HashSet<Vector2Int>();
        connectedCities.Add(cities[0]);

        while (connectedCities.Count < cities.Count)
        {
            float minDistance = float.MaxValue;
            Vector2Int nextCity = Vector2Int.zero;
            Vector2Int sourceCity = Vector2Int.zero;

            foreach (Vector2Int connected in connectedCities)
            {
                foreach (Vector2Int city in cities)
                {
                    if (!connectedCities.Contains(city))
                    {
                        float dist = Vector2.Distance(connected, city);
                        if (dist < minDistance)
                        {
                            minDistance = dist;
                            nextCity = city;
                            sourceCity = connected;
                        }
                    }
                }
            }

            // Connect the next closest city
            BuildRoad(sourceCity, nextCity);
            connectedCities.Add(nextCity);
        }
    }

    private void ConnectTownsToNearestCity()
    {
        foreach (Vector2Int town in towns)
        {
            float minDistance = float.MaxValue;
            Vector2Int nearestCity = Vector2Int.zero;

            // Find the nearest city
            foreach (Vector2Int city in cities)
            {
                float dist = Vector2.Distance(town, city);
                if (dist < minDistance && dist <= roadMaxDistance)
                {
                    minDistance = dist;
                    nearestCity = city;
                }
            }

            // If a city is within range, connect to it
            if (minDistance <= roadMaxDistance)
            {
                BuildRoad(town, nearestCity);
            }
        }
    }

    private void BuildRoad(Vector2Int start, Vector2Int end)
    {
        // Simple implementation for now - just a straight line
        // We'll replace this with proper A* later
        int xDiff = end.x - start.x;
        int yDiff = end.y - start.y;
        int steps = Mathf.Max(Mathf.Abs(xDiff), Mathf.Abs(yDiff));

        for (int i = 0; i <= steps; i++)
        {
            float t = steps == 0 ? 0 : (float)i / steps;
            int x = Mathf.RoundToInt(Mathf.Lerp(start.x, end.x, t));
            int y = Mathf.RoundToInt(Mathf.Lerp(start.y, end.y, t));

            Vector2Int roadTile = new Vector2Int(x, y);
            roadTiles.Add(roadTile);
        }
    }
}
