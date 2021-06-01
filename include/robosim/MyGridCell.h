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

#ifndef __MY_GRID_CELL__
#define __MY_GRID_CELL__

namespace mygridcell {

static const char *occupancyTypes[] = {"OBSTACLE", "EMPTY", "ROBOT", "RED", "BLUE", "GREEN", "UNKNOWN"};
static const char *occupancySymbols[] = {" # ", " _ ", " * ", " r ", " b ", " g ", " ? "};

enum OccupancyType : int {
    OBSTACLE,
    EMPTY,
    ROBOT,
    RED,
    BLUE,
    GREEN,
    UNKNOWN
};

template <typename T>
class MyGridCell {
  private:
    T cellType;

  public:
    MyGridCell(T type) { cellType = type; }
    T getCellType() { return cellType; }
    void setCellType(T type) { cellType = type; }
    void setEmpty() { cellType = mygridcell::OccupancyType::EMPTY; }
    bool isEmpty() { return cellType == OccupancyType::EMPTY; }
    const char *toString() { return occupancySymbols[cellType]; }

    static MyGridCell getEmptyCell() {
        MyGridCell emptyCell(OccupancyType::EMPTY);
        return emptyCell;
    }
};

} // namespace mygridcell

#endif // !__MY_GRID_CELL__
