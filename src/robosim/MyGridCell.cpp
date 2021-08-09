/**
 * @file MyGridCell.h
 * @author Robert Azzopardi-Yashi (robertazzopardi@icloud.com)
 * @brief One of the cells in the grid environment
 * @version 0.1
 * @date 2021-05-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "MyGridCell.h"

namespace mygridcell {

namespace {
static const char *occupancySymbols[] = {"#", "_", "*", "r", "b", "g", "?"};
}

MyGridCell::MyGridCell() : cellType(OccupancyType::EMPTY) {}

OccupancyType MyGridCell::getCellType() const { return cellType; }

void MyGridCell::setCellType(OccupancyType type) { cellType = type; }

void MyGridCell::setEmpty() { cellType = mygridcell::OccupancyType::EMPTY; }

bool MyGridCell::isEmpty() const { return cellType == OccupancyType::EMPTY; }

const char *MyGridCell::toString() const { return occupancySymbols[cellType]; }

}  // namespace mygridcell
