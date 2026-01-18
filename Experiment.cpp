#include <iostream>
#include <vector>
#include <random>
#include <fstream>
#include "MapExporter.h"
#include <queue>
#include <chrono>
#include <unordered_set>
#include "JPSGrid.h"
#include "JPS.h"
#include "JPSTools.h"
#include "AStarSecond.h"

constexpr int GRID_WIDTH = 512;
constexpr int GRID_HEIGHT = 512;

std::vector<std::vector<int>> GenerateRandomizedMap(double density, int seed)
{
    std::vector<std::vector<int>> grid(GRID_HEIGHT, std::vector<int>(GRID_WIDTH, 0));
    std::mt19937 gen(seed);
    std::bernoulli_distribution distribution(density);

    //assign randomly to every cell in the grid either an obstacle(1) or empty(0)
    for (int y = 0; y < GRID_HEIGHT; y++)
    {
        for (int x = 0; x < GRID_WIDTH; x++)
        {
            if (distribution(gen))
            {
                grid[y][x] = 1;
            }
            else
            {
                grid[y][x] = 0;
            }
        }
    }
    //start and goal cell must be empty!
    grid[0][0] = 0;
    grid[GRID_HEIGHT - 1][GRID_WIDTH - 1] = 0;
    grid[1][1] = 0;                         
    grid[GRID_HEIGHT - 2][GRID_WIDTH - 2] = 0;  

    return grid;
}

double CalculateDensity(const std::vector<std::vector<int>>& grid)
{
    int totalObstacles = 0;

    for (size_t y = 0; y < grid.size(); y++)
    {
        for (size_t x = 0; x < grid.size(); x++)
        {
            if (grid[y][x] == 1)
            {
                totalObstacles++;
            }
        }
    }

    return static_cast<double> (totalObstacles) / (GRID_HEIGHT * GRID_WIDTH);
}

void AdjustDensityToTarget(std::vector<std::vector<int>>& grid , double targetDenisty, int seed = 42)
{
    //calculate the difference between the current density and the target density
    double currentDensity = CalculateDensity(grid);
    int targetCount = static_cast<int>(targetDenisty * GRID_WIDTH * GRID_HEIGHT);
    int currentCount = static_cast<int>(currentDensity * GRID_WIDTH * GRID_HEIGHT);
    int difference = targetCount - currentCount;//the obstcles needed to be added

    //creating a random generater so the new cells we're going to add/remove to be distributed randomly

    std::mt19937 gen(seed);

    if (difference > 0)//add more obstacles if difference is bigger
    {
        //we loop though all the cells, find the ones that are empty(excluding the start and end cell), and put them in a list:
        std::vector<std::pair<int, int>> emptyTiles;
        for (int y = 0; y < GRID_HEIGHT; y++)
        {
            for (int x = 0; x < GRID_WIDTH; x++)
            {
                if (grid[y][x] == 0 && !(x == 0 && y == 0) && !(x == GRID_WIDTH - 1 && y == GRID_HEIGHT - 1))
                {
                    emptyTiles.push_back({ x, y });
                }
            }
        }

        //shuffle the empty list to randomly select the cells you want to convert to obstacles
        std::shuffle(emptyTiles.begin(), emptyTiles.end(), gen);
        //use std::min so if difference is bigger than available cells, it will try to access the emtpy tiles which will lead to an error!
        for (int i = 0; i < std::min(difference, (int)emptyTiles.size()); i++)
        {
            //colour them black
            int x = emptyTiles[i].first;
            int y = emptyTiles[i].second;
            grid[y][x] = 1;
        }
    }
    else if (difference < 0)//remove obstacles if difference is less
    {
        std::vector<std::pair<int, int>> obstacleTiles;

        for (int y = 0; y < GRID_HEIGHT; y++)
        {
            for (int x = 0; x < GRID_WIDTH; x++)
            {
                if (grid[y][x] == 1 && !(x == 0 && y == 0) && !(x == GRID_WIDTH - 1 && y == GRID_HEIGHT - 1))
                {
                    obstacleTiles.push_back({ x,y });
                }
            }
        }

        std::shuffle(obstacleTiles.begin(), obstacleTiles.end(), gen);
        for (int i = 0; i < std::min(-difference, (int)obstacleTiles.size()); i++)
        {
            //colour them black
            int x = obstacleTiles[i].first;
            int y = obstacleTiles[i].second;
            grid[y][x] = 0;
        }
    }
}

void SaveToPPMFile(const std::vector<std::vector<int>>& grid , const std::string& filename)
{
    std::ofstream out(filename, std::ios::binary);
    out << "P6\n" << GRID_WIDTH << " " << GRID_HEIGHT << "\n255\n";

    std::vector<std::vector<unsigned char>> r(GRID_HEIGHT, std::vector<unsigned char>(GRID_WIDTH, 255));
    std::vector<std::vector<unsigned char>> g(GRID_HEIGHT, std::vector<unsigned char>(GRID_WIDTH, 255));
    std::vector<std::vector<unsigned char>> b(GRID_HEIGHT, std::vector<unsigned char>(GRID_WIDTH, 255));

    for (int y = 0; y < GRID_HEIGHT; y++)
    {
        for (int x = 0; x < GRID_WIDTH; x++)
        {
            if (grid[y][x] == 1)//if a cell is obsatcle
            {
                //color it black
                r[y][x] = 0;
                g[y][x] = 0;
                b[y][x] = 0;
            }
        }
    }

    for (int y = 0; y < GRID_HEIGHT; y++)
    {
        for (int x = 0; x < GRID_WIDTH; x++)
        {
            out.put(r[y][x]);
            out.put(g[y][x]);
            out.put(b[y][x]);
        }
    }
}

void SaveJPSResultToPPM(const std::vector<std::vector<int>>& grid, const std::vector<Location>& path, const std::unordered_map<Location, Location>& came_from, const std::string& filename)
{
    std::ofstream out(filename, std::ios::binary);
    out << "P6\n" << GRID_WIDTH << " " << GRID_HEIGHT << "\n255\n";

    std::vector<std::vector<unsigned char>> r(GRID_HEIGHT, std::vector<unsigned char>(GRID_WIDTH, 255));
    std::vector<std::vector<unsigned char>> g(GRID_HEIGHT, std::vector<unsigned char>(GRID_WIDTH, 255));
    std::vector<std::vector<unsigned char>> b(GRID_HEIGHT, std::vector<unsigned char>(GRID_WIDTH, 255));

    for (int y = 0; y < GRID_HEIGHT; y++)
    {
        for (int x = 0; x < GRID_WIDTH; x++)
        {
            if (grid[y][x] == 1)
            {
                r[y][x] = 0;
                g[y][x] = 0;
                b[y][x] = 0;
            }
        }
    }

    for (const auto& pair : came_from)
    {
        Location loc = pair.first;
        if (loc.x >= 0 && loc.x < GRID_WIDTH && loc.y >= 0 && loc.y < GRID_HEIGHT)
        {
            r[loc.y][loc.x] = 100;
            g[loc.y][loc.x] = 100;
            b[loc.y][loc.x] = 255;
        }
    }

    for (const auto& loc : path)
    {
        if (loc.x >= 0 && loc.x < GRID_WIDTH && loc.y >= 0 && loc.y < GRID_HEIGHT)
        {
            r[loc.y][loc.x] = 255;
            g[loc.y][loc.x] = 0;
            b[loc.y][loc.x] = 255;
        }
    }

    r[1][1] = 0;
    g[1][1] = 255;
    b[1][1] = 0;

    r[GRID_HEIGHT - 2][GRID_WIDTH - 2] = 255;
    g[GRID_HEIGHT - 2][GRID_WIDTH - 2] = 255;
    b[GRID_HEIGHT - 2][GRID_WIDTH - 2] = 0;

    for (int y = 0; y < GRID_HEIGHT; y++)
    {
        for (int x = 0; x < GRID_WIDTH; x++)
        {
            out.put(r[y][x]);
            out.put(g[y][x]);
            out.put(b[y][x]);
        }
    }

    out.close();
}



void AppendMetricsToCSV(const std::string& filename, long long timeMs, size_t nodesExpanded, const std::string& mapName)
{
    std::ifstream infile(filename);
    bool fileExists = infile.good();
    infile.close();

    std::ofstream outfile(filename, std::ios::app);

    if (!fileExists)
    {
        outfile << "MapName,TimeMs,NodesExpanded\n";
    }

    outfile << mapName << "," << timeMs << "," << nodesExpanded << "\n";
    outfile.close();
}



//A* - Pre-made maps

//int main()
//{
//    Map maze = LoadMapFile("Resources/MazeMaps/maze512-1-0.map");
//    //Map maze = LoadMapFile("Resources/StarCraftMaps/Archipelago.map"); //clustered
//    //Map maze = LoadMapFile("Resources/RoomMaps/8room_000.map");  //room
//
//    AStar::AStar<uint32_t, true> astar;
//    astar.setWorldSize({ GRID_WIDTH, GRID_HEIGHT });
//    astar.setHeuristic(AStar::Heuristic::euclidean);
//    astar.setDiagonalMovement(true);
//    astar.setHeuristicWeight(1);
//    astar.setMouvemementCost(1);
//
//    for (int y = 0; y < GRID_HEIGHT; ++y)
//    {
//        for (int x = 0; x < GRID_WIDTH; ++x)
//        {
//            if (!maze.IsPassable(x, y))
//            {
//                astar.addObstacle({ x, y });
//            }
//        }
//    }
//
//    AStar::Vec2i start{ 1, 1 };
//    AStar::Vec2i end{ GRID_WIDTH - 1, GRID_HEIGHT - 1 };
//
//    if (!maze.IsPassable(start.x, start.y) || !maze.IsPassable(end.x, end.y))
//    {
//        return 1;
//    }
//
//    std::vector<AStar::Vec2i> expandedNodes;
//
//    astar.setDebugCurrentNode([&expandedNodes](AStar::Node<uint32_t>* node) 
//    {
//        expandedNodes.push_back(node->pos);
//    });
//
//    auto startTime = std::chrono::high_resolution_clock::now();
//    auto path = astar.findPath(start, end);
//    auto endTime = std::chrono::high_resolution_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//
//    if (path.empty())
//    {
//        return 1;
//    }
//
//    AppendMetricsToCSV("astar_maze_metrics.csv", duration.count(), expandedNodes.size(), "Maze");
//    SaveToPPMFile("[AStar]Maze.ppm", maze, path, expandedNodes);
//
//    return 0;
//}



//A* - Random

//int main()
//{
//    const int seed = 5;
//    double targetDensity = 0.45;
//
//    std::vector<std::vector<int>> grid = GenerateRandomizedMap(targetDensity, seed);
//    AdjustDensityToTarget(grid, targetDensity);
//
//    AStar::AStar<uint32_t, true> astar;
//    astar.setWorldSize({ GRID_WIDTH, GRID_HEIGHT });
//    astar.setHeuristic(AStar::Heuristic::euclidean);
//    astar.setDiagonalMovement(true);
//    astar.setHeuristicWeight(1);
//    astar.setMouvemementCost(1);
//
//    for (int y = 0; y < GRID_HEIGHT; ++y)
//    {
//        for (int x = 0; x < GRID_WIDTH; ++x)
//        {
//            if (grid[y][x] == 1) 
//            {
//                astar.addObstacle({ x, y });
//            }
//        }
//    }
//
//    AStar::Vec2i start{ 1, 1 };
//    AStar::Vec2i end{ GRID_WIDTH - 1, GRID_HEIGHT - 1 };
//
//    if (grid[start.y][start.x] == 1 || grid[end.y][end.x] == 1)
//    {
//        return 1;
//    }
//
//    std::vector<AStar::Vec2i> expandedNodes;
//    astar.setDebugCurrentNode([&expandedNodes](AStar::Node<uint32_t>* node)
//    {
//        expandedNodes.push_back(node->pos);
//    });
//
//    auto startTime = std::chrono::high_resolution_clock::now();
//    auto path = astar.findPath(start, end);
//    auto endTime = std::chrono::high_resolution_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//
//    if (path.empty())
//    {
//        return 1;
//    }
//
//    AppendMetricsToCSV("astar_randomized_metrics_50.csv", duration.count(), expandedNodes.size(), "Randomized_50");
//    SaveToPPMFileRandomized("[AStar]Random.ppm", grid, path, expandedNodes);
//
//    return 0;
//}



//JPS - Random

//int main()
//{
//    double targetDensity = 0.25f;
//    int seed = 5;
//
//    Location start{ 1, 1 };
//    Location goal{ GRID_WIDTH - 1, GRID_HEIGHT - 1 };
//
//    auto grid = GenerateRandomizedMap(targetDensity, seed);
//    AdjustDensityToTarget(grid, targetDensity, seed);
//
//    std::unordered_set<Location> walls;
//    for (int y = 0; y < GRID_HEIGHT; y++)
//    {
//        for (int x = 0; x < GRID_WIDTH; x++)
//        {
//            if (grid[y][x] == 1)
//            {
//                walls.insert(Location{ x, y });
//            }
//        }
//    }
//
//    Grid jpsGrid(GRID_WIDTH, GRID_HEIGHT, walls);
//
//    if (!jpsGrid.passable(start) || jpsGrid.passable(goal))
//    {
//        return 1;
//    }
//
//    auto jpsStart = std::chrono::high_resolution_clock::now();
//    auto came_from = jps(jpsGrid, start, goal, Tool::euclidean);
//    auto jpsEnd = std::chrono::high_resolution_clock::now();
//    auto jpsDuration = std::chrono::duration_cast<std::chrono::milliseconds>(jpsEnd - jpsStart);
//
//    auto path = Tool::reconstruct_path(start, goal, came_from);
//
//    if (path.empty())
//    {
//        return 1;
//    }
//
//    SaveJPSResultToPPM(grid, path, came_from, "[JPS]16x16.ppm");
//    AppendMetricsToCSV("jps_randomized_50_metrics.csv", jpsDuration.count(), came_from.size(), "jps_50");
//
//    return 0;
//}



//JPS - Pre-made maps

//int main()
//{
//    //Map maze = LoadMapFile("Resources/MazeMaps/maze512-1-0.map"); //maze
//    //Map maze = LoadMapFile("Resources/StarCraftMaps/Archipelago.map"); //clustered
//    Map maze = LoadMapFile("Resources/RoomMaps/8room_000.map"); //room
//
//    std::unordered_set<Location> walls;
//    for (int y = 0; y < maze.height; y++)
//    {
//        for (int x = 0; x < maze.width; x++)
//        {
//            if (!maze.IsPassable(x, y))
//            {
//                walls.insert(Location{ x, y });
//            }
//        }
//    }
//    
//    Grid jpsGrid(maze.width, maze.height, walls);
//
//    Location start{ 1, 1 };
//    Location goal{ maze.width - 1, maze.height - 1 };
//
//    if (!jpsGrid.passable(start) || jpsGrid.passable(goal))
//    {
//        return 1;
//    }
//
//    auto jpsStart = std::chrono::high_resolution_clock::now();
//    auto came_from = jps(jpsGrid, start, goal, Tool::euclidean);
//    auto jpsEnd = std::chrono::high_resolution_clock::now();
//    auto jpsDuration = std::chrono::duration_cast<std::chrono::milliseconds>(jpsEnd - jpsStart);
//
//    auto path = Tool::reconstruct_path(start, goal, came_from);
//
//    SaveJPSToPPM("MazeMapExample.ppm", maze, path, came_from);
//    AppendMetricsToCSV("jps_room_metrics2.csv", jpsDuration.count(), came_from.size(), "Room");
//
//    return 0;
//}