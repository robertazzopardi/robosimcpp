#include "ArenaModel.h"
#include "MyGridCell.h"
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>

#define SIZE 800

using arenamodel::ArenaModel;
using mygridcell::MyGridCell;
using mygridcell::OccupancyType;

static const std::regex reg("\\s*,\\s*");

ArenaModel::ArenaModel(const char *configFileName, int arenaWidth, int arenaHeight) {
    arenaWidthInCells = arenaWidth;
    arenaHeightInCells = arenaHeight;
    this->configFileName = configFileName;

    cellWidth = SIZE / (float)arenaHeightInCells;

    for (int i = 0; i < arenaHeight; i++) {
        std::vector<MyGridCell<OccupancyType>> row;
        for (int j = 0; j < arenaWidth; j++) {
            row.push_back(MyGridCell<OccupancyType>::getEmptyCell());
        }
        grid.push_back(row);
    }

    readConfig();
}

ArenaModel::~ArenaModel() {
}

int ArenaModel::getArenaWidthInCells() {
    return arenaWidthInCells;
}

int ArenaModel::getArenaHeightInCells() {
    return arenaHeightInCells;
}

bool ArenaModel::setOccupancy(int colPos, int rowPos, OccupancyType type) {
    bool result = false;

    // Check the bounds of the col and row pos
    if ((rowPos >= 0) && (rowPos < (int)grid.size())) {
        if ((colPos >= 0) && (colPos < (int)grid.size())) {
            // Only change the occupancy of a cell if it is empty
            // Note, separate methods will be used when modelling the robot
            if (grid.at(rowPos).at(colPos).isEmpty()) {
                grid.at(rowPos).at(colPos).setCellType(type);
                result = true;
            }
        }
    }

    return result;
}

OccupancyType ArenaModel::getOccupancy(int colPos, int rowPos) {
    // Check the bounds of the col and row pos
    if ((rowPos >= 0) && (rowPos < (int)grid.size())) {
        if ((colPos >= 0) && (colPos < (int)grid.size())) {
            return grid.at(rowPos).at(colPos).getCellType();
        }
    }

    return OccupancyType::UNKNOWN;
}

bool ArenaModel::parseConfigLine(std::string line) {
    std::vector<std::string> tokens = tokenize(line);

    int colPos;
    int rowPos;

    std::stringstream ss;
    ss << tokens.at(0);
    ss >> colPos;
    ss.clear();
    ss << tokens.at(1);
    ss >> rowPos;

    std::string occTypeStr = tokens.at(2);
    OccupancyType occ = OccupancyType::EMPTY;

    if (occTypeStr == "OBSTACLE") {
        occ = OccupancyType::OBSTACLE;
        numOfObstacles++;
    } else if (occTypeStr == "BLUE") {
        occ = OccupancyType::BLUE;
        numOfBlues++;
    } else if (occTypeStr == "GREEN") {
        occ = OccupancyType::GREEN;
        numOfGreens++;
    } else if (occTypeStr == "RED") {
        occ = OccupancyType::RED;
        numOfReds++;
    }

    if (occ != OccupancyType::EMPTY && !setOccupancy(colPos, rowPos, occ)) {
        std::cout << "Error in parseConfigLine setting (" << colPos << "," << rowPos << ")" << std::endl;
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

    std::vector<std::string> vec(iter, end);

    return vec;
}

bool ArenaModel::readConfig() {
    std::fstream file;
    file.open(configFileName);

    std::string line;

    bool result = true;

    // Use a while loop together with the getline() function to read the file line by line
    while (getline(file, line)) {
        // Output the text from the file
        if (line != "")
            result = parseConfigLine(line);
    }

    file.close();

    return result;
}

int ArenaModel::getObstacleCount() {
    return numOfObstacles;
}
int ArenaModel::getRedCount() {
    return numOfReds;
}
int ArenaModel::getGreenCount() {
    return numOfGreens;
}
int ArenaModel::getBlueCount() {
    return numOfBlues;
}
float ArenaModel::getCellWidth() {
    return cellWidth;
}

std::string ArenaModel::toString() {
    std::string result = "Arena: " + std::to_string(getArenaWidthInCells()) + " x " + std::to_string(getArenaHeightInCells()) + "\n";

    for (auto var : grid) {
        for (auto c : var) {
            result += c.toString();
        }
        result += "\n";
    }
    return result;
}
