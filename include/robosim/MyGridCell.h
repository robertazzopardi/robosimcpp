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

#ifndef __MY_GRID_CELL_H__
#define __MY_GRID_CELL_H__

#include <map>

namespace mygridcell {

static const char *occupancySymbols[] = {" # ", " _ ", " * ", " r ",
                                         " b ", " g ", " ? "};

enum OccupancyType { OBSTACLE, EMPTY, ROBOT, RED, BLUE, GREEN, UNKNOWN };

template <typename T> class MyGridCell {
  private:
    T cellType;

  public:
    explicit MyGridCell(T type) : cellType(type) {}

    MyGridCell() : cellType(EMPTY) {}

    T getCellType() const { return cellType; }

    void setCellType(T type) { cellType = type; }

    void setEmpty() { cellType = mygridcell::OccupancyType::EMPTY; }

    bool isEmpty() { return cellType == OccupancyType::EMPTY; }

    const char *toString() const { return occupancySymbols[cellType]; }

    // static MyGridCell getEmptyCell() {
    //     return static_cast<MyGridCell>(OccupancyType::EMPTY);
    // }
};

using Cell = MyGridCell<OccupancyType>;

} // namespace mygridcell

#endif // !__MY_GRID_CELL_H__
