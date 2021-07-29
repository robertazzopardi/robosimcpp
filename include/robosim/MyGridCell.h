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

namespace mygridcell {

static const char *occupancySymbols[] = {"#", "_", "*", "r", "b", "g", "?"};

enum OccupancyType { OBSTACLE, EMPTY, ROBOT, RED, BLUE, GREEN, UNKNOWN };

class MyGridCell {
  private:
    OccupancyType cellType;

  public:
    explicit MyGridCell(OccupancyType);

    MyGridCell();

    OccupancyType getCellType() const;

    void setCellType(OccupancyType);

    void setEmpty();

    bool isEmpty() const;

    const char *toString() const;
};

} // namespace mygridcell

#endif // !__MY_GRID_CELL_H__
