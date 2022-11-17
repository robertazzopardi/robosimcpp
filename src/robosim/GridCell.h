#pragma once

namespace gridcell
{

enum OccupancyType
{
    OBSTACLE,
    EMPTY,
    ROBOT,
    RED,
    BLUE,
    GREEN,
    UNKNOWN
};

class GridCell
{
  private:
    OccupancyType cellType;

  public:
    explicit GridCell(OccupancyType);

    GridCell();

    OccupancyType getCellType() const;

    void setCellType(OccupancyType);

    void setEmpty();

    bool isEmpty() const;

    const char *toString() const;
};

} // namespace gridcell
