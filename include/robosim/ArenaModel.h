#pragma once

#include "MyGridCell.h"
#include <vector>

namespace arenamodel
{

    using Row = std::vector<mygridcell::MyGridCell>;
    using Grid = std::vector<Row>;

    struct ConfigLine
    {
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

}
