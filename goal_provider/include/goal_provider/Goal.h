#include <stdlib.h>

/**
  A class to represent a Goal.

  @author Enda McCauley
*/
class Goal
{
  public:
    /**
      Field to store the x location of this Goal
    */
    double x;

    /**
      Field to store the y location of this Goal
    */
    double y;

    /**
      Field to store the distance of this Goal from the robot
    */
    double distanceFromRobot;

    Goal();
    Goal(double, double, double);

    bool operator> (Goal& g)
    {
        return distanceFromRobot > g.distanceFromRobot;
    }

    bool operator>= (Goal& g)
    {
        return distanceFromRobot > g.distanceFromRobot;
    }

    bool operator< (Goal& g)
    {
        return distanceFromRobot < g.distanceFromRobot;
    }

    bool operator<= (Goal& g)
    {
        return distanceFromRobot <= g.distanceFromRobot;
    }

    bool operator== (Goal& g)
    {
        double deltaX = 0.0;
        double deltaY = 0.0;

        if (x >= 0 && g.x >= 0)
        {
            deltaX = x - g.x;
        }
        else if (x < 0 && g.x < 0)
        {
            deltaX = abs(x) - abs(g.x);
        }
        else
        {
            return false;
        }

        if (y >= 0 && g.y >= 0)
        {
            deltaY = y - g.y;
        }
        else if (y < 0 && g.y < 0)
        {
            deltaY = abs(y) - abs(g.y);
        }
        else
        {
            return false;
        }

        return (deltaX < 0.001 && deltaX > -0.001) && (deltaY < 0.001 && deltaY > -0.001);
    }

    bool operator!= (Goal& g)
    {
        return !(distanceFromRobot == g.distanceFromRobot);
    }
};
