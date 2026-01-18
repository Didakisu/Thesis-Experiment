#pragma once
#include <vector>
#include <string>
#include "AStarSecond.h"
#include <unordered_map>
#include "JPSGrid.h"

struct Map
{
	int width;
	int height;
	std::vector<std::string> grid;

	bool IsPassable(int x, int y) const
	{
		if (x < 0 || y < 0 || x >= width || y >= height)//check if out of bounds
		{
			return false;
		}
		
		return grid[y][x] != '@' && grid[y][x] != 'T'; //for room and clustered map
	}
};

Map LoadMapFile(const std::string fileName);
void SaveToPPMFile(const std::string& fileName, const Map& map, const std::vector<AStar::Vec2i>& path, const std::vector<AStar::Vec2i>& expandedNodes);
void SaveToPPMFileRandomized(const std::string& fileName, const std::vector<std::vector<int>>& grid, const std::vector<AStar::Vec2i>& path, const std::vector<AStar::Vec2i>& expandedNodes);
void SaveJPSToPPM(const std::string& fileName, const Map& map, const std::vector<Location>& path, const std::unordered_map<Location, Location>& came_from);