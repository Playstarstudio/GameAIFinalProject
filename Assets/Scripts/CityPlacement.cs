using UnityEngine;
using System.Collections.Generic;
using UnityEngine.Tilemaps;

public class CityPlacement : MonoBehaviour
{
    // Function to find suitable city locations on a Perlin noise map
    public Tilemap tileMap;
    public Tile baseTile;
    public static List<Vector2Int> FindCityLocations(float[,] noiseMap, bool traversable, int cityCount, int minDistanceBetweenCities)
    {
        List<Vector2Int> cityLocations = new List<Vector2Int>();
        int width = noiseMap.GetLength(0);
        int height = noiseMap.GetLength(1);

        // First, identify all possible city locations (5x5 traversable areas)
        List<Vector2Int> possibleLocations = FindAllTraversableAreas(noiseMap, traversable, 5);

        // If we don't have enough possible locations, return what we can
        if (possibleLocations.Count < cityCount)
        {
            Debug.LogWarning("Not enough traversable areas for all cities!");
            return possibleLocations;
        }

        // Sort possible locations by elevation (assuming higher is better for cities)
        possibleLocations.Sort((a, b) => noiseMap[b.x, b.y].CompareTo(noiseMap[a.x, a.y]));

        // Start with the highest location
        cityLocations.Add(possibleLocations[0]);
        possibleLocations.RemoveAt(0);

        // Find remaining cities with minimum distance requirement
        while (cityLocations.Count < cityCount && possibleLocations.Count > 0)
        {
            // Find the next best location that's far enough from existing cities
            for (int i = 0; i < possibleLocations.Count; i++)
            {
                Vector2Int candidate = possibleLocations[i];
                bool validLocation = true;

                foreach (Vector2Int city in cityLocations)
                {
                    float distance = Vector2Int.Distance(city, candidate);
                    if (distance < minDistanceBetweenCities)
                    {
                        validLocation = false;
                        break;
                    }
                }

                if (validLocation)
                {
                    cityLocations.Add(candidate);
                    possibleLocations.RemoveAt(i);
                    break;
                }
            }

            // If we couldn't find a valid location, relax the distance requirement slightly
            if (cityLocations.Count < cityCount)
            {
                minDistanceBetweenCities = Mathf.Max(minDistanceBetweenCities - 5, 10);
                Debug.Log($"Reducing minimum distance between cities to {minDistanceBetweenCities}");
            }
        }

        return cityLocations;
    }

    // Finds all 5x5 areas where all cells are traversable
    private static List<Vector2Int> FindAllTraversableAreas(float[,] noiseMap, bool traversable, int areaSize)
    {
        List<Vector2Int> traversableAreas = new List<Vector2Int>();
        int width = noiseMap.GetLength(0);
        int height = noiseMap.GetLength(1);

        for (int x = 0; x < width - areaSize; x++)
        {
            for (int y = 0; y < height - areaSize; y++)
            {
                if (IsAreaTraversable(noiseMap, x, y, areaSize, traversable))
                {
                    // Use the center of the area as the city location
                    traversableAreas.Add(new Vector2Int(x + areaSize / 2, y + areaSize / 2));
                }
            }
        }

        return traversableAreas;
    }

    // Checks if a square area is completely traversable
    private static bool IsAreaTraversable(float[,] noiseMap, int startX, int startY, int size, bool traversable)
    {
        for (int x = startX; x < startX + size; x++)
        {
            for (int y = startY; y < startY + size; y++)
            {
                // If any cell in the area is below the threshold, it's not traversable
                if (noiseMap[x, y] == traversable)
                {
                    return false;
                }
            }
        }
        return true;
    }

    // Visual debug function to draw city locations in the scene
    public static void DebugDrawCityLocations(float[,] noiseMap, List<Vector2Int> cityLocations, float cellSize)
    {
        foreach (Vector2Int location in cityLocations)
        {
            float worldX = location.x * cellSize;
            float worldZ = location.y * cellSize;
            float height = noiseMap[location.x, location.y] * 10f; // Scale for visibility

            Vector3 center = new Vector3(worldX, height, worldZ);

            // Draw a red sphere at the city location
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(center, 2f);

            // Draw a yellow wire cube around the 5x5 area
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(center, new Vector3(5 * cellSize, 0.1f, 5 * cellSize));
        }
    }
}