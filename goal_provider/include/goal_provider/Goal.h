#include <stdlib.h>
#include <math.h>

#include <geometry_msgs/Pose.h>

/**
  A class to represent a Goal

  @author Enda McCauley
  @date November 21st 2013
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
      Field to store the distance of this goal from the robot
    */
    double distanceFromRobot;

    /**
      Constructs a new empty Goal object
    */
    Goal();

    /**
      Constructs a new Goal object
      @param x The x location of this Goal
      @param y The y location of this Goal
      @param distanceFromRobot The distance of this Goal from the robot
    */
    Goal(double, double, double);

    friend Goal operator-(const Goal& g1, const Goal& g2);
    friend Goal operator-(const Goal& g, const geometry_msgs::Pose& p);

    /**
      Override the > operator based on the distance between this and the other goal
      @param g The other goal
    */
    bool operator> (Goal& g)
    {
        return distanceFromRobot > g.distanceFromRobot;
    }

    /**
      Override the >= operator based on the distance between this and the other goal
      @param g The other goal
    */
    bool operator>= (Goal& g)
    {
        return distanceFromRobot > g.distanceFromRobot;
    }

    /**
      Override the < operator based on the distance between this and the other goal
      @param g The other goal
    */
    bool operator< (Goal& g)
    {
        return distanceFromRobot < g.distanceFromRobot;
    }

    /**
      Override the <= operator based on the distance between this and the other goal
      @param g The other goal
    */
    bool operator<= (Goal& g)
    {
        return distanceFromRobot <= g.distanceFromRobot;
    }

    /**
      Override the != operator based on the distance between this and the other goal
      @param g The other goal
    */
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

        return (deltaX < 0.5 && deltaX > -0.5) && (deltaY < 0.5 && deltaY > -0.5);
    }

    /**
      Override the != operator based on the distance between this and the other goal
      @param g The other goal
    */
    bool operator!= (Goal& g)
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
            return true;
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
            return true;
        }

        return (deltaX > 0.001 && deltaX < -0.001) && (deltaY > 0.001 && deltaY < -0.001);
    }
};
