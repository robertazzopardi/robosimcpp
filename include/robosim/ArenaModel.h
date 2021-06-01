/**
 * @file ArenaModel.h
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief The model for the arena, includeing a parser for including obstacles and other artifacts in the arena model
 * @version 0.1
 * @date 2021-05-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __ARENA_MODEL__
#define __ARENA_MODEL__

#include <iosfwd>
#include <vector>

namespace mygridcell {
enum OccupancyType : int;

template <typename T>
class MyGridCell;
} // namespace mygridcell

namespace arenamodel {

class ArenaModel {
  private:
    const char *configFileName;
    int arenaWidthInCells;
    int arenaHeightInCells;

    std::vector<std::vector<mygridcell::MyGridCell<mygridcell::OccupancyType>>> grid;

    bool setOccupancy(int, int, mygridcell::OccupancyType);
    bool parseConfigLine(std::string);
    bool readConfig();

    std::vector<std::string> tokenize(std::string);

    int numOfObstacles;
    int numOfReds;
    int numOfGreens;
    int numOfBlues;

    float cellWidth;

  public:
    ArenaModel(const char *, int, int);
    ~ArenaModel();

    int getArenaWidthInCells();
    int getArenaHeightInCells();

    mygridcell::OccupancyType getOccupancy(int, int);
    std::string toString();

    int getObstacleCount();
    int getRedCount();
    int getGreenCount();
    int getBlueCount();

    float getCellWidth();
};

} // namespace arenamodel

#endif // !__ARENA_MODEL__
