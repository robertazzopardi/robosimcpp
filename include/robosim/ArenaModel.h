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

#include "MyGridCell.h"
#include <iosfwd>
#include <vector>

namespace arenamodel {

using Cell = mygridcell::MyGridCell<mygridcell::OccupancyType>;
using Row = std::vector<Cell>;
using Grid = std::vector<Row>;

struct ConfigLine {
    int row;
    int col;
    mygridcell::OccupancyType occ;
};

class ArenaModel {
  private:
    static arenamodel::ConfigLine tokenize(std::string);
    static void parseConfigFile(const char *);

    ArenaModel(const char *, int, int);
    ~ArenaModel();

  public:
    static void makeModel(const char *);
    static void makeModel(int, int);

    static int arenaWidthInCells;
    static int arenaHeightInCells;
    static float cellWidth;

    static bool setOccupancy(int, int, mygridcell::OccupancyType);
    static mygridcell::OccupancyType getOccupancy(int, int);
    static void toString();

    static Grid grid;
};

} // namespace arenamodel

#endif // !__ARENA_MODEL_H__
