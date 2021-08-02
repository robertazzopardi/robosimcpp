#include "MyGridCell.h"

namespace mygridcell {

MyGridCell::MyGridCell(OccupancyType type) : cellType(type) {}

MyGridCell::MyGridCell() : cellType(OccupancyType::EMPTY) {}

OccupancyType MyGridCell::getCellType() const { return cellType; }

void MyGridCell::setCellType(OccupancyType type) { cellType = type; }

void MyGridCell::setEmpty() { cellType = mygridcell::OccupancyType::EMPTY; }

bool MyGridCell::isEmpty() const { return cellType == OccupancyType::EMPTY; }

const char *MyGridCell::toString() const { return occupancySymbols[cellType]; }

} // namespace mygridcell