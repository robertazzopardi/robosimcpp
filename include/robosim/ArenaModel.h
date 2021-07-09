/**
 * @file ArenaModel.h
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief The model for the arena, includeing a parser for including obstacles
 * and other artifacts in the arena model
 * @version 0.1
 * @date 2021-05-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __ARENA_MODEL_H__
#define __ARENA_MODEL_H__

#include <iosfwd>
#include <vector>

namespace mygridcell {
enum OccupancyType : int;

template <typename T> class MyGridCell;
} // namespace mygridcell

namespace arenamodel {

using Grid =
    std::vector<std::vector<mygridcell::MyGridCell<mygridcell::OccupancyType>>>;

class ArenaModel {
  private:
    const char *configFileName;
    int arenaWidthInCells;
    int arenaHeightInCells;
    float cellWidth;
    Grid grid;

    bool setOccupancy(int, int, mygridcell::OccupancyType);
    bool parseConfigLine(std::string, int *, int *);
    bool readConfig();
    std::vector<std::string> tokenize(std::string);

  public:
    ArenaModel(const char *, int, int);
    ~ArenaModel();

    mygridcell::OccupancyType getOccupancy(int, int);
    std::string toString();
    int getArenaWidthInCells();
    int getArenaHeightInCells();
    float getCellWidth();

    Grid getGrid();
};

} // namespace arenamodel

#endif // !__ARENA_MODEL_H__
