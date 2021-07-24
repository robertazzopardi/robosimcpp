/**
 * @file ArenaModel.cpp
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief
 * @version 0.1
 * @date 2021-07-07
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "ArenaModel.h"
#include "MyGridCell.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

using mygridcell::OccupancyType;

namespace arenamodel {

float cellWidth = 0;
Grid grid;

namespace {

constexpr auto SIZE = 800;
const std::regex reg("\\s*,\\s*");

struct ConfigLine {
    int row;
    int col;
    mygridcell::OccupancyType occ;
};

ConfigLine tokenize(std::string str) {
    // Get an iterator after filtering through the regex
    std::sregex_token_iterator iter(str.begin(), str.end(), reg, -1);

    std::sregex_token_iterator end;

    std::vector<std::string> tokens(iter, end);

    auto occTypeStr = tokens[2];
    auto occ = OccupancyType::EMPTY;

    if (occTypeStr == "OBSTACLE") {
        occ = OccupancyType::OBSTACLE;
    } else if (occTypeStr == "BLUE") {
        occ = OccupancyType::BLUE;
    } else if (occTypeStr == "GREEN") {
        occ = OccupancyType::GREEN;
    } else if (occTypeStr == "RED") {
        occ = OccupancyType::RED;
    }

    return {std::stoi(tokens[0]), std::stoi(tokens[1]), occ};
}

void parseConfigFile(const char *filePath) {
    std::fstream file;
    file.open(filePath);

    std::string line;

    std::vector<ConfigLine> lines;
    while (getline(file, line)) {
        // Output the text from the file
        if (!line.empty()) {
            lines.push_back(tokenize(line));
        }
    }

    auto maxRow = std::max_element(lines.begin(), lines.end(),
                                   [](const auto &a, const auto &b) {
                                       return a.row < b.row;
                                   })
                      ->row +
                  1;
    auto maxCol = std::max_element(lines.begin(), lines.end(),
                                   [](const auto &a, const auto &b) {
                                       return a.col < b.col;
                                   })
                      ->col +
                  1;

    grid.resize(maxCol, Row(maxRow));

    cellWidth = (float)SIZE / grid[0].size();

    for (auto l : lines) {
        setOccupancy(l.row, l.col, l.occ);
    }
}

} // namespace

void makeModel(const char *configFileName) { parseConfigFile(configFileName); }

void makeModel(int width, int height) {
    grid.resize(width, Row(height));

    cellWidth = SIZE / grid.size();

    // Add boundaries to empty grid
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            if (x == 0 || y == 0 || x == width - 1 || y == height - 1) {
                grid[x][y].setCellType(OccupancyType::OBSTACLE);
            }
        }
    }
}

bool setOccupancy(int row, int col, OccupancyType type) {
    auto size = static_cast<int>(grid.size());

    // Check the bounds of the col and row pos
    if ((row >= 0 && row <= size) && (col >= 0 && col <= size)) {
        // Only change the occupancy of a cell if it is empty
        // Note, separate methods will be used when modelling the robot
        auto cell = &grid[col][row];

        if (cell->isEmpty()) {
            cell->setCellType(type);
            return true;
        }
    }

    return false;
}

OccupancyType getOccupancy(int row, int col) {
    static auto size = static_cast<int>(grid.size());

    // Check the bounds of the col and row pos
    if ((row >= 0 && row <= size) && (col >= 0 && col <= size)) {
        return grid[col][row].getCellType();
    }

    return OccupancyType::UNKNOWN;
}

void toString() {
    std::cout << "Arena: " << grid[0].size() << " x " << grid.size() << "\n";

    for (auto var : grid) {
        for (auto c : var) {
            std::cout << c.toString() << " ";
        }
        std::cout << "\n";
    }
}

} // namespace arenamodel
