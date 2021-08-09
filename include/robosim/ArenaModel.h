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
#include <vector>

namespace arenamodel {

using Row = std::vector<mygridcell::MyGridCell>;
using Grid = std::vector<Row>;

struct ConfigLine {
    int row;
    int col;
    mygridcell::OccupancyType occ;
};

extern Grid grid;
extern float cellWidth;

void makeModel(const char *);
void makeModel(int, int);

void setOccupancy(ConfigLine);

mygridcell::OccupancyType getOccupancy(int, int);
void toString();

}  // namespace arenamodel

#endif  // !__ARENA_MODEL_H__
