using UnityEngine;
using System.Collections.Generic;
using System;

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

    //comment the following code if console output needs to be showcased
    [Header("Debug Settings")]
    public bool verboseLogging = false;

    void Start()
    {
        GenerateMap();
    }

    private int regenerationAttempts = 0;
    private const int maxRegenerationAttempts = 5;

    public void GenerateMap()
    {
        // Only log at the beginning if it's verbose or the first attempt; can be commneted out to show all the outputs
        if (verboseLogging || regenerationAttempts == 0)
        {
            Debug.Log($"Starting map generation (Attempt {regenerationAttempts + 1}/{maxRegenerationAttempts})");
        }

        // Reset regeneration counter if this is a manual call
        if (regenerationAttempts >= maxRegenerationAttempts)
        {
            Debug.LogError("Too many regeneration attempts, stopping to prevent stack overflow");
            regenerationAttempts = 0;
            return;
        }

        regenerationAttempts++;

        // Generate noise maps as initially planned
        float[,] noiseMap1 = GenerateNoiseMap(mapWidth, mapHeight, seed, scale1, octaves1, persistence, lacunarity, offset);
        float[,] noiseMap2 = GenerateNoiseMap(mapWidth, mapHeight, seed, scale2, octaves2, persistence, lacunarity, offset);
        float[,] noiseMap3 = GenerateNoiseMap(mapWidth, mapHeight, seed, scale3, octaves3, persistence, lacunarity, offset);

        // Draw terrain first
        mapDisplay.DrawNoiseMap(noiseMap1, noiseMap2, noiseMap3);

        // Place cities
        PlaceCities();

        // Verify connectivity
        if (!VerifyConnectivity())
        {
            // Only log if verbose or final attempt
            if (verboseLogging || regenerationAttempts >= maxRegenerationAttempts)
            {
                Debug.LogWarning($"Map doesn't meet connectivity requirements, regenerating... (Attempt {regenerationAttempts})");
            }
            seed++; // Change seed to get different results
            GenerateMap();
            return;
        }

        // Generate road network
        try
        {
            GenerateRoads();
        }
        catch (Exception e)
        {
            Debug.LogError($"Error generating roads: {e.Message}");
            Debug.LogWarning("Continuing with generation despite road errors");
        }

        // Draw cities, towns, and roads on the tilemap
        mapDisplay.DrawSettlements(cities, towns, roadTiles);

        // Debug visualization
        VisualizeSettlements();

        // Only log success on the final successful attempt
        Debug.Log($"Successfully generated map on attempt {regenerationAttempts}/{maxRegenerationAttempts}");

        // Reset counter after successful generation
        regenerationAttempts = 0;
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
        cities.Clear();
        int attempts = 0;
        int maxAttempts = 1000;

        while (cities.Count < numberOfCities && attempts < maxAttempts)
        {
            attempts++;

            // Random position
            int x = UnityEngine.Random.Range(0, mapWidth);
            int y = UnityEngine.Random.Range(0, mapHeight);
            Vector2Int pos = new Vector2Int(x, y);

            // Check if 5x5 area around this position is suitable
            if (IsSuitableAreaForCity(x, y, 5))
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
                    // Mark the city's area in the map data
                    // MarkCityArea(pos); // Implement this if needed
                }
            }
        }

        // Replace below for the output comments
        //Debug.Log($"Placed {cities.Count} cities after {attempts} attempts");

        if (verboseLogging || regenerationAttempts >= maxRegenerationAttempts)
        {
            Debug.Log($"Placed {cities.Count} cities after {attempts} attempts");
        }


        // Warning if we couldn't place all cities, then don't regenerate
        if (cities.Count < numberOfCities)
        {
            Debug.LogWarning("Could not place enough cities. Continuing with fewer cities.");
        }
    }


    private bool IsSuitableAreaForCity(int centerX, int centerY, int areaSize)
    {
        int radius = areaSize / 2;

        // Check boundaries
        if (centerX - radius < 0 || centerX + radius >= mapWidth ||
            centerY - radius < 0 || centerY + radius >= mapHeight)
        {
            return false;
        }

        // Instead of requiring all cells to be perfect, just check if most are traversable
        int traversableCount = 0;
        int totalCells = 0;

        for (int y = centerY - radius; y <= centerY + radius; y++)
        {
            for (int x = centerX - radius; x <= centerX + radius; x++)
            {
                totalCells++;
                float value = mapDisplay.GetNoiseValue(x, y);
                MapDisplay.TerrainType terrain = mapDisplay.GetTerrainType(x, y);

                if (terrain.isTraversable)
                {
                    traversableCount++;
                }
            }
        }

        // If at least 70% of cells are traversable, consider this area suitable
        return (float)traversableCount / totalCells >= 0.7f;
    }

    private void MarkCityArea(Vector2Int cityPos)
    {
        // Mark a 3x3 diamond as city
        int[] dx = { 0, 1, 0, -1, 0 };
        int[] dy = { 0, 0, 1, 0, -1 };

        for (int i = 0; i < dx.Length; i++)
        {
            int x = cityPos.x + dx[i];
            int y = cityPos.y + dy[i];

            // Skip if out of bounds
            if (x < 0 || x >= mapWidth || y < 0 || y >= mapHeight)
                continue;

            // Mark as city
            // We'll need to implement this in MapDisplay
        }
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
            int x = UnityEngine.Random.Range(0, mapWidth);
            int y = UnityEngine.Random.Range(0, mapHeight);
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

        // If no cities, nothing to connect
        if (cities.Count == 0)
        {
            Debug.LogWarning("No cities to connect with roads");
            return;
        }

        // Create roads between cities using Dijkstra's algorithm
        for (int i = 0; i < cities.Count; i++)
        {
            // Skip if only one city
            if (cities.Count <= 1)
            {
                break;
            }

            Vector2Int sourceCity = cities[i];
            Vector2Int targetCity = FindClosestReachableCity(sourceCity, i);

            if (targetCity != Vector2Int.zero)
            {
                BuildRoadWithDijkstra(sourceCity, targetCity);
            }
            else
            {
                Debug.LogWarning($"Could not find reachable city from city {i}. Not all cities are connected.");
            }
        }

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

    private bool VerifyConnectivity()
    {
        // Create distance map from a random traversable cell
        Vector2Int startPos = FindRandomTraversableCell();
        Dictionary<Vector2Int, float> distanceMap = CreateDistanceMap(startPos);

        // Count all traversable cells
        int totalTraversableCells = CountTraversableCells();

        // Count reachable cells from distance map
        int reachableCells = distanceMap.Count;

        // Calculate connectivity percentage
        float connectivityPercentage = (float)reachableCells / totalTraversableCells;

        // Replace with below in case of detailed console outputs
        //Debug.Log($"Connectivity: {connectivityPercentage * 100}% ({reachableCells}/{totalTraversableCells})");

        if (verboseLogging || regenerationAttempts >= maxRegenerationAttempts)
        {
            Debug.Log($"Connectivity: {connectivityPercentage * 100}% ({reachableCells}/{totalTraversableCells})");
        }

        // Verify city connectivity
        if (!VerifyCityConnectivity(distanceMap))
        {
            // Replace with below in case of detailed console outputs
            //Debug.LogWarning("Cities are not all reachable from each other");

            if (verboseLogging || regenerationAttempts >= maxRegenerationAttempts)
            {
                Debug.LogWarning("Cities are not all reachable from each other");
            }
            return false;
        }

        return connectivityPercentage >= 0.8f; // 80% requirement
    }

    private int CountTraversableCells()
    {
        int count = 0;
        for (int y = 0; y < mapHeight; y++)
        {
            for (int x = 0; x < mapWidth; x++)
            {
                float value = mapDisplay.GetNoiseValue(x, y);
                MapDisplay.TerrainType terrain = mapDisplay.GetTerrainType(x, y);
                if (terrain.isTraversable)
                {
                    count++;
                }
            }
        }
        return count;
    }

    private Vector2Int FindRandomTraversableCell()
    {
        int maxAttempts = 1000;
        int attempts = 0;

        while (attempts < maxAttempts)
        {
            attempts++;
            int x = UnityEngine.Random.Range(0, mapWidth);
            int y = UnityEngine.Random.Range(0, mapHeight);

            float value = mapDisplay.GetNoiseValue(x, y);
            MapDisplay.TerrainType terrain = mapDisplay.GetTerrainType(x, y);

            if (terrain.isTraversable)
            {
                return new Vector2Int(x, y);
            }
        }

        // Default fallback
        Debug.LogError("Could not find traversable cell, using first city position");
        return cities[0];
    }

    private Dictionary<Vector2Int, float> CreateDistanceMap(Vector2Int startPos)
    {
        // Dijkstra's algorithm
        Dictionary<Vector2Int, float> distanceMap = new Dictionary<Vector2Int, float>();
        PriorityQueue<Vector2Int, float> frontier = new PriorityQueue<Vector2Int, float>();

        distanceMap[startPos] = 0;
        frontier.Enqueue(startPos, 0);

        while (frontier.Count > 0)
        {
            Vector2Int current = frontier.Dequeue();
            float currentDistance = distanceMap[current];

            // Check all four neighbors
            int[] dx = { 1, 0, -1, 0 };
            int[] dy = { 0, 1, 0, -1 };

            for (int i = 0; i < 4; i++)
            {
                Vector2Int neighbor = new Vector2Int(current.x + dx[i], current.y + dy[i]);

                // Skip if out of bounds
                if (neighbor.x < 0 || neighbor.x >= mapWidth ||
                    neighbor.y < 0 || neighbor.y >= mapHeight)
                {
                    continue;
                }

                // Skip if not traversable
                float value = mapDisplay.GetNoiseValue(neighbor.x, neighbor.y);
                MapDisplay.TerrainType terrain = mapDisplay.GetTerrainType(neighbor.x, neighbor.y);
                if (!terrain.isTraversable)
                {
                    continue;
                }

                // Calculate new distance (including terrain cost)
                float terrainCost = 1.0f + value * roadCostMultiplier; // Higher values are more costly
                float newDistance = currentDistance + terrainCost;

                // If found shorter path, update
                if (!distanceMap.ContainsKey(neighbor) || newDistance < distanceMap[neighbor])
                {
                    distanceMap[neighbor] = newDistance;
                    frontier.Enqueue(neighbor, newDistance);
                }
            }
        }

        return distanceMap;
    }

    private bool VerifyCityConnectivity(Dictionary<Vector2Int, float> distanceMap)
    {
        // Check if cities list is empty
        if (cities.Count == 0)
        {
            Debug.LogWarning("No cities to verify connectivity for");
            return true; // Return true to avoid regeneration
        }

        // Check if all cities are reachable
        foreach (Vector2Int city in cities)
        {
            if (!distanceMap.ContainsKey(city))
            {
                return false;
            }
        }

        // If only one city, nothing to connect
        if (cities.Count <= 1)
        {
            return true;
        }

        // Check if all cities have the same reachable cells count
        Dictionary<Vector2Int, float> firstCityMap = CreateDistanceMap(cities[0]);
        int firstCityReachableCount = firstCityMap.Count;

        for (int i = 1; i < cities.Count; i++)
        {
            Dictionary<Vector2Int, float> cityMap = CreateDistanceMap(cities[i]);
            if (cityMap.Count != firstCityReachableCount)
            {
                return false;
            }
        }

        return true;
    }

    private Vector2Int FindClosestReachableCity(Vector2Int sourceCity, int skipIndex)
    {
        // If no other cities, return zero vector
        if (cities.Count <= 1)
        {
            return Vector2Int.zero;
        }

        // Create distance map from source city
        Dictionary<Vector2Int, float> distanceMap = CreateDistanceMap(sourceCity);

        Vector2Int closestCity = Vector2Int.zero;
        float minDistance = float.MaxValue;

        for (int i = 0; i < cities.Count; i++)
        {
            if (i == skipIndex) continue; // Skip self

            Vector2Int targetCity = cities[i];

            // Check if city is reachable
            if (distanceMap.ContainsKey(targetCity))
            {
                float distance = distanceMap[targetCity];
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestCity = targetCity;
                }
            }
        }

        return closestCity;
    }

    private void BuildRoadWithDijkstra(Vector2Int start, Vector2Int end)
    {
        // Create distance map from start
        Dictionary<Vector2Int, float> distanceMap = CreateDistanceMap(start);

        // Check if end is reachable
        if (!distanceMap.ContainsKey(end))
        {
            Debug.LogWarning("Cannot reach target city, skipping road");
            return;
        }

        // Trace path from end to start
        Vector2Int current = end;

        while (current != start)
        {
            roadTiles.Add(current);

            // Find neighbor with lowest distance
            Vector2Int nextStep = Vector2Int.zero;
            float lowestDistance = float.MaxValue;

            int[] dx = { 1, 0, -1, 0 };
            int[] dy = { 0, 1, 0, -1 };

            for (int i = 0; i < 4; i++)
            {
                Vector2Int neighbor = new Vector2Int(current.x + dx[i], current.y + dy[i]);

                // Skip if out of bounds
                if (neighbor.x < 0 || neighbor.x >= mapWidth ||
                    neighbor.y < 0 || neighbor.y >= mapHeight)
                {
                    continue;
                }

                // Skip if not traversable
                float value = mapDisplay.GetNoiseValue(neighbor.x, neighbor.y);
                MapDisplay.TerrainType terrain = mapDisplay.GetTerrainType(neighbor.x, neighbor.y);
                if (!terrain.isTraversable)
                {
                    continue;
                }

                // Check if this neighbor is in our distance map
                if (distanceMap.ContainsKey(neighbor) && distanceMap[neighbor] < lowestDistance)
                {
                    lowestDistance = distanceMap[neighbor];
                    nextStep = neighbor;
                }
            }

            if (nextStep == Vector2Int.zero)
            {
                Debug.LogError("Could not find next step in path, breaking");
                break;
            }

            current = nextStep;
        }

        // Add start point
        roadTiles.Add(start);
    }
}

public class PriorityQueue<T, TPriority> where TPriority : IComparable<TPriority>
{
    private List<(T item, TPriority priority)> elements = new List<(T, TPriority)>();

    public int Count => elements.Count;

    public void Enqueue(T item, TPriority priority)
    {
        elements.Add((item, priority));
        // Keep list sorted by priority
        elements.Sort((a, b) => a.priority.CompareTo(b.priority));
    }

    public T Dequeue()
    {
        if (elements.Count == 0)
        {
            throw new InvalidOperationException("Queue is empty");
        }

        T item = elements[0].item;
        elements.RemoveAt(0);
        return item;
    }
}