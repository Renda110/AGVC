#include <goal_provider/Goal.h>
#include <geometry_msgs/Pose.h>

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

/**
  Override the - operator to allow subtracting one Goal from another
  @param g1 Goal 1
  @param g2 The Goal to subtract from goal 1
  @return A new Goal holding the result of the subtraction
*/
Goal operator- (const Goal& g1, const Goal& g2)
{
    Goal g;

    g.x = g1.x - g2.x;
    g.y = g1.y - g2.y;

    return g;
}

/**
  Override the - operator to allow subtracting a Pose from a Goal
  @param g Goal 1
  @param p The Pose to subtract from goal 1
  @return A new Goal holding the result of the subtraction
*/
Goal operator- (const Goal& g, const geometry_msgs::Pose& p)
{
    Goal toReturn;

    toReturn.x = g.x - p.position.x;
    toReturn.y = g.y - p.position.y;

    return toReturn;
}
