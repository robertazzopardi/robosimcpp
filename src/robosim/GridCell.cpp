#include "GridCell.h"

namespace gridcell
{

namespace
{

static const char *occupancySymbols[] = {"#", "_", "*", "r", "b", "g", "?"};

}

GridCell::GridCell() : cellType(OccupancyType::EMPTY)
{
}

OccupancyType GridCell::getCellType() const
{
    return cellType;
}

void GridCell::setCellType(OccupancyType type)
{
    cellType = type;
}

void GridCell::setEmpty()
{
    cellType = gridcell::OccupancyType::EMPTY;
}

bool GridCell::isEmpty() const
{
    return cellType == OccupancyType::EMPTY;
}

const char *GridCell::toString() const
{
    return occupancySymbols[cellType];
}

} // namespace gridcell
