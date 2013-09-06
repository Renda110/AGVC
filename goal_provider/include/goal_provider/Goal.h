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
      return distanceFromRobot >= g.distanceFromRobot;
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
      return distanceFromRobot == g.distanceFromRobot;
    }

    bool operator!= (Goal& g)
    {
      return distanceFromRobot != g.distanceFromRobot;
    }
};
