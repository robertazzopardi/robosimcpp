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
#include <fstream>
#include <iostream>
#include <regex>
#include <string>

using arenamodel::ArenaModel;
using mygridcell::OccupancyType;
using Cell = mygridcell::MyGridCell<OccupancyType>;

constexpr auto SIZE = 800;
const std::regex reg("\\s*,\\s*");

ArenaModel::ArenaModel(const char *configFileName, int arenaWidth,
                       int arenaHeight)
    : grid(arenaHeight, std::vector<Cell>(arenaWidth, Cell::getEmptyCell())) {

    arenaWidthInCells = arenaWidth;
    arenaHeightInCells = arenaHeight;
    this->configFileName = configFileName;

    cellWidth = SIZE / arenaHeightInCells;

    readConfig();
}

ArenaModel::~ArenaModel() {}

int ArenaModel::getArenaWidthInCells() { return arenaWidthInCells; }

int ArenaModel::getArenaHeightInCells() { return arenaHeightInCells; }

float ArenaModel::getCellWidth() { return cellWidth; }

bool ArenaModel::setOccupancy(int colPos, int rowPos, OccupancyType type) {
    auto result = false;

    auto size = static_cast<int>(grid.size());

    // Check the bounds of the col and row pos
    if ((rowPos >= 0 && rowPos < size) && (colPos >= 0 && colPos < size)) {
        // Only change the occupancy of a cell if it is empty
        // Note, separate methods will be used when modelling the robot
        auto cell = &grid.at(rowPos).at(colPos);
        if (cell->isEmpty()) {
            cell->setCellType(type);
            result = true;
        }
    }

    return result;
}

OccupancyType ArenaModel::getOccupancy(int colPos, int rowPos) {
    auto size = static_cast<int>(grid.size());

    // Check the bounds of the col and row pos
    if ((rowPos >= 0 && rowPos < size) && (colPos >= 0 && colPos < size)) {
        return grid.at(rowPos).at(colPos).getCellType();
    }

    return OccupancyType::UNKNOWN;
}

bool ArenaModel::parseConfigLine(std::string line) {
    auto tokens = tokenize(line);

    int colPos = std::stoi(tokens.at(0));
    int rowPos = std::stoi(tokens.at(1));

    auto occTypeStr = tokens.at(2);
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

    if (occ != OccupancyType::EMPTY && !setOccupancy(colPos, rowPos, occ)) {
        std::cout << "Error in parseConfigLine setting (" << colPos << ","
                  << rowPos << ")" << std::endl;
        return false;
    }
    return true;
}

std::vector<std::string> ArenaModel::tokenize(std::string str) {
    // Get an iterator after filtering through the regex
    std::sregex_token_iterator iter(str.begin(), str.end(), reg, -1);
    // Keep a dummy end iterator - Needed to construct a vector
    // using (start, end) iterators.
    std::sregex_token_iterator end;

    return {iter, end};
}

bool ArenaModel::readConfig() {
    std::fstream file;
    file.open(configFileName);

    std::string line;

    auto result = true;

    // Use a while loop together with the getline() function to read the file
    // line by line
    while (getline(file, line)) {
        // Output the text from the file
        if (line != "")
            result = parseConfigLine(line);
    }

    file.close();

    return result;
}

std::string ArenaModel::toString() {
    auto result = "Arena: " + std::to_string(getArenaWidthInCells()) + " x " +
                  std::to_string(getArenaHeightInCells()) + "\n";

    for (auto var : grid) {
        for (auto c : var) {
            result += c.toString();
        }
        result += "\n";
    }
    return result;
}
