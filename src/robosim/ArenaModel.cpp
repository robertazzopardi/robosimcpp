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

static void initGrid(int width, int height, const int dim) {
    grid.resize(width, Row(height));
    cellWidth = SIZE / dim;
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

    int width = 0, height = 0;
    for (auto var : lines) {
        if (var.col > width)
            width = var.col + 1;
        if (var.row > height)
            height = var.row + 1;
    }

    initGrid(width, height, height);

    std::for_each(lines.begin(), lines.end(), setOccupancy);
}

static constexpr auto isInBounds(const int row, const int col) {
    return (row >= 0 && row <= static_cast<int>(grid.size())) &&
           (col >= 0 && col <= static_cast<int>(grid.size()));
}

} // namespace

void makeModel(const char *configFileName) { parseConfigFile(configFileName); }

void makeModel(int width, int height) {
    initGrid(width, height, width);

    for (int i = 0; i < height; i++) {
        grid[i][0].setCellType(OccupancyType::OBSTACLE);
        grid[i][height - 1].setCellType(OccupancyType::OBSTACLE);
    }

    for (int i = 1; i < width - 1; i++) {
        grid[0][i].setCellType(OccupancyType::OBSTACLE);
        grid[width - 1][i].setCellType(OccupancyType::OBSTACLE);
    }
}

void setOccupancy(ConfigLine line) {
    // Check the bounds of the col and row pos
    if (isInBounds(line.row, line.col)) {
        // Only change the occupancy of a cell if it is empty
        // Note, separate methods will be used when modelling the robot
        auto cell = &grid[line.col][line.row];

        if (cell->isEmpty()) {
            cell->setCellType(line.occ);
            // return true;
        }
    }

    // return false;
}

OccupancyType getOccupancy(int row, int col) {
    // Check the bounds of the col and row pos
    if (isInBounds(row, col)) {
        return grid[col][row].getCellType();
    }

    return OccupancyType::UNKNOWN;
}

void toString() {
    std::string arenaModel = " Arena " + std::to_string(grid[0].size()) +
                             " x " + std::to_string(grid.size()) + "\n";

    for (auto var : grid) {
        for (auto c : var) {
            arenaModel += c.toString();
            arenaModel += " ";
        }
        arenaModel += "\n";
    }

    std::cout << arenaModel << std::endl;
}

} // namespace arenamodel
