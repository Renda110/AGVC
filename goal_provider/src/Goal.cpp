#include <goal_provider/Goal.h>

/**
  Constucts an empty Point object
*/
Goal::Goal()
{
    this->x = 0;
    this->y = 0;
    this->distanceFromRobot = 0;
}

/**
  Constructs a new Goal object
  @param x The x location of this Goal
  @param y The y location of this Goal
  @param distanceFromRobot The distance of this Goal from the robot
*/
Goal::Goal(double x, double y, double distanceFromRobot)
{
  this->x = x;
  this->y = y;
  this->distanceFromRobot = distanceFromRobot;
}

Goal operator- (const Goal& g1, const Goal& g2)
{
    Goal g;

    g.x = g1.x - g2.x;
    g.y = g1.y - g2.y;

    return g;
}
