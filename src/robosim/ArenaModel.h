#pragma once

#include "GridCell.h"
#include <cstdint>
#include <vector>

namespace arenamodel
{

using Row = std::vector<gridcell::GridCell>;
using Grid = std::vector<Row>;

struct ConfigLine
{
    int32_t row;
    int32_t col;
    gridcell::OccupancyType occ;
};

extern Grid grid;
extern float cellWidth;

void makeModel(const char *);
void makeModel(int, int);

void setOccupancy(const ConfigLine &);

gridcell::OccupancyType getOccupancy(int, int);
void toString();

} // namespace arenamodel
