#pragma once

namespace mygridcell
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

class MyGridCell
{
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
