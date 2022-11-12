#include "MyGridCell.h"

namespace mygridcell
{

    namespace
    {
        static const char *occupancySymbols[] = {"#", "_", "*", "r", "b", "g", "?"};
    }

    MyGridCell::MyGridCell() : cellType(OccupancyType::EMPTY) {}

    OccupancyType MyGridCell::getCellType() const { return cellType; }

    void MyGridCell::setCellType(OccupancyType type) { cellType = type; }

    void MyGridCell::setEmpty() { cellType = mygridcell::OccupancyType::EMPTY; }

    bool MyGridCell::isEmpty() const { return cellType == OccupancyType::EMPTY; }

    const char *MyGridCell::toString() const { return occupancySymbols[cellType]; }

} // namespace mygridcell
