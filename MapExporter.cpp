#include "MapExporter.h"
#include <fstream>
#include <iostream>
#include "JPSGrid.h"
#include <unordered_map>

constexpr int GRID_WIDTH = 512;
constexpr int GRID_HEIGHT = 512;

Map LoadMapFile(const std::string fileName)
{
	Map map;
	map.width = GRID_WIDTH;   
	map.height = GRID_HEIGHT;

	std::ifstream file(fileName);

	if (!file.is_open())
	{
		std::cout << "Can't open file" << std::endl;
	}

	std::string line;

	//there r 4 lines i don't need
	std::getline(file, line);
	std::getline(file, line);
	std::getline(file, line);
	std::getline(file, line);

	while (std::getline(file, line))
	{
		if (!line.empty())
		{
			map.grid.push_back(line);
		}
	}

	file.close();

	return map;
}

void SaveToPPMFile(const std::string& fileName, const Map& map, const std::vector<AStar::Vec2i>& path, const std::vector<AStar::Vec2i>& expandedNodes)
{
    std::ofstream out(fileName, std::ios::binary);
    if (!out.is_open())
    {
        return;
    }

    out << "P6\n" << map.width << " " << map.height << "\n255\n";

    for (int y = 0; y < map.height; ++y)
    {
        for (int x = 0; x < map.width; ++x)
        {
            unsigned char r, g, b;

            if (!map.IsPassable(x, y))
            {
                r = g = b = 0;
            }
            else
            {
                r = g = b = 255;
            }

            if (std::any_of(expandedNodes.begin(), expandedNodes.end(), [x, y](const AStar::Vec2i& n) { return n.x == x && n.y == y; }))
            {
                r = 100;
                g = 100;
                b = 255;
            }

            if (std::any_of(path.begin(), path.end(), [x, y](const AStar::Vec2i& p) { return p.x == x && p.y == y; }))
            {
                r = 255;
                g = 0;
                b = 255;
            }

            if (x == 1 && y == 1)
            {
                r = 0;
                g = 255;
                b = 0;
            }

            if (x == map.width - 1 && y == map.height - 1)
            {
                r = 255;
                g = 255;
                b = 0;
            }

            out.put(r);
            out.put(g);
            out.put(b);
        }
    }

    out.close();
}


void SaveToPPMFileRandomized(const std::string& fileName, const std::vector<std::vector<int>>& grid, const std::vector<AStar::Vec2i>& path, const std::vector<AStar::Vec2i>& expandedNodes)
{
    std::ofstream out(fileName, std::ios::binary);
    if (!out.is_open())
    {
        return;
    }

    int height = static_cast<int>(grid.size());
    int width = static_cast<int>(grid[0].size());

    out << "P6\n" << width << " " << height << "\n255\n";

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            unsigned char r, g, b;
            bool onPath = false;

            if (grid[y][x] == 1) 
                r = g = b = 0;
            else
                r = g = b = 255;

            for (const auto& n : expandedNodes)
            {
                if (n.x == x && n.y == y)
                {
                    r = 100;
                    g = 100;
                    b = 255;
                    break;
                }
            }

            for (const auto& p : path)
            {
                if (p.x == x && p.y == y)
                {
                    r = 255;
                    g = 0;
                    b = 255;
                    break;
                }
            }

            if (x == 1 && y == 1)
            {
                r = 0;
                g = 255;
                b = 0;
            }

            if (x == width - 1 && y == height - 1)
            {
                r = 255;
                g = 255;
                b = 0;
            }

            out.put(r);
            out.put(g);
            out.put(b);
        }
    }

    out.close();
}


void SaveJPSToPPM(const std::string& fileName, const Map& map, const std::vector<Location>& path, const std::unordered_map<Location, Location>& came_from)
{
    std::ofstream out(fileName, std::ios::binary);
    if (!out.is_open())
    {
        return;
    }

    out << "P6\n" << map.width << " " << map.height << "\n255\n";

    for (int y = 0; y < map.height; ++y)
    {
        for (int x = 0; x < map.width; ++x)
        {
            unsigned char r, g, b;
            bool onPath = false;

            if (!map.IsPassable(x, y))
            {
                r = g = b = 0;
            }
            else
            {
                r = g = b = 255;
            }

            bool isExpanded = false;
            Location currentLoc{ x, y };
            if (came_from.find(currentLoc) != came_from.end())
            {
                isExpanded = true;
            }

            if (isExpanded)
            {
                r = 100;
                g = 100;
                b = 255;
            }

            for (std::size_t i = 0; i < path.size(); i++)
            {
                const Location& p = path[i];
                if (p.x == x && p.y == y)
                {
                    onPath = true;
                    break;
                }
            }

            if (onPath)
            {
                r = 255;
                g = 0;
                b = 255;
            }

            if (x == 1 && y == 1)
            {
                r = 0;
                g = 255;
                b = 0;
            }

            if (x == map.width - 1 && y == map.height - 1)
            {
                r = 255;
                g = 255;
                b = 0;
            }

            out.put(r);
            out.put(g);
            out.put(b);
        }
    }

    out.close();
}