#include <iostream>
#include <fcntl.h>
#include <fstream>
#include <math.h>
#include <string>
#include <sstream>
#include <termios.h>
#include <time.h>
#include <vector>
#include <unistd.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <gps_common/conversions.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>

#include <goal_provider/AutonomousStatus.h>
#include <goal_provider/Goal.h>

using namespace goal_provider;
using namespace ros;
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define PI 3.14159265
#define d2r PI/180

/**
    The GoalProvider class.

    This class provides goal data to the move_base navigation stack. A typical goal is simply a GPS position

    @author Enda McCauley
    @date October 2nd 2013
*/
class GoalProvider
{
public:
    GoalProvider() : mb("move_base", true)
    {
        NodeHandle node;
        NodeHandle privateNode("~");

        autonomousStatusPublisher = privateNode.advertise<goal_provider::AutonomousStatus>("status", 1000);

        while(!mb.waitForServer(ros::Duration(1.0)))
        {
            ROS_INFO("\033[2;33mGoalProvider: Waiting for the Movebase action server to come up\033[0m\n");

            publishStatus(AutonomousStatus::WAITING, "Waiting for the Movebase action server to come up");
        }

        setStartingPosition = false;
        setStartingOrientation = false;
        sentFirstGoal = false;
        startTime = ros::Time::now();
        currentCallbackCount = 0;
        goalAttemptCounter = 0;
        goalAcceptanceCounter = 0;
        angleOffNorth = 0.0;

        ROS_INFO("\033[2;32mGoalProvider: Initialized at time: %d.%d\033[0m\n", startTime.sec, startTime.nsec);

        privateNode.param("callbacks_till_goal_update", callbacksUntilGoalUpdate, 1000);
        privateNode.param("competition_length", competitionTimeLimit, 100);
        privateNode.param("time_cutoff", cutoffTime, 80);
        privateNode.param("goal_attempt_limit", goalAttemptLimit, 5);
        privateNode.param("testing", testing, false);

        ROS_INFO("\033[2;32mGoalProvider: Callbacks until goal update set to: %d\033[0m\n", callbacksUntilGoalUpdate);
        ROS_INFO("\033[2;32mGoalProvider: Competition Length set to: %d\033[0m\n", competitionTimeLimit);
        ROS_INFO("\033[2;32mGoalProvider: Time cutoff set to: %d\033[0m\n", cutoffTime);
        ROS_INFO("\033[2;32mGoalProvider: Goal attempt limit set to: %d\033[0m\n", goalAttemptLimit);
        ROS_INFO("\033[2;32mGoalProvider: Testing set to: %d\033[0m\n", (testing) ? 1 : 0);

        gpsSubscriber = node.subscribe("/gpsfix", 1000, &GoalProvider::gpsCallback, this);
        imuSubscriber = node.subscribe("/raw_imu", 1000, &GoalProvider::imuCallback, this);
        hlPoseSubscriber = node.subscribe("localization_pose", 1000, &GoalProvider::hlPoseCallback, this);
        movebaseFeedbackSubscriber = node.subscribe("move_base/feedback", 1000, &GoalProvider::movebaseFeedbackCallback, this);
        movebaseStatusSubscriber = node.subscribe("move_base/status", 1000, &GoalProvider::movebaseStatusCallback, this);

        hlOdomPublisher = privateNode.advertise<nav_msgs::Odometry>("odom", 1000);
    }

private:
    /**
      Field to store the start time
    */
    Time startTime;

    /**
      Field to store an Actionlib object. It allows us to send goals to move_base and monitor them
      all in one system
    */
    MoveBaseClient mb;

    /**
      Field to store the status of this node
    */
    AutonomousStatus status;

    /**
      Field to store the GPS subscriber
    */
    Subscriber gpsSubscriber;

    /**
      Field to store the hector_localization/localization_pose subscriber
    */
    Subscriber hlPoseSubscriber;

    /**
      Field to store the IMU subscriber
    */
    Subscriber imuSubscriber;

    /**
      Field to store the movebase feedback subscriber
    */
    Subscriber movebaseFeedbackSubscriber;

    /**
      Field to store the movebase status subscriber
    */
    Subscriber movebaseStatusSubscriber;

    /**
      Field to store the publisher that we publish our status on
    */
    Publisher autonomousStatusPublisher;

    /**
      Field to store the publisher that the converted Hector Localization odometry is published on
    */
    Publisher hlOdomPublisher;

    /**
      Field to store the list of Points we need to go to
    */
    list<Goal> coordsList;

    /**
      Field to store the starting position of the robot based on GPS converted into UTM
    */
    Goal gpsUTMOrigin;

    /**
      Field to store our current goal
    */
    Goal goal;

    /**
      Field to store whether or not we have set our starting absolute position
    */
    bool setStartingPosition;

    /**
      Field to store whether or not we have set our starting absolute orientation
    */
    bool setStartingOrientation;

    /**
      Field to store if we are testing without GPS or not
    */
    bool testing;

    /**
      Field to store if we have sent the first goal or not
    */
    bool sentFirstGoal;

    /**
      Field to store the angle off of magnetic north that the robot started at
    */
    double angleOffNorth;

    /**
      Field to store the number of position callbacks that need to be made before we update our goal
    */
    int callbacksUntilGoalUpdate;

    /**
      Field to store the maximum number of goal attempts before choosing a new one
    */
    int goalAttemptLimit;

    /**
      Field to store the number of attempts made to get to the current goal
    */
    int goalAttemptCounter;

    /**
      Field to store the available time to complete the course
    */
    int competitionTimeLimit;

    /**
      Field to store the cutoff time before we start working to home
    */
    int cutoffTime;

    /**
      Field to store the current number of position callbacks
    */
    int currentCallbackCount;

    /**
      Field to store whether or not the goal has been accepted
    */
    int goalAcceptanceCounter;

    /**
      Callback for the GPS
      @param odom The odometry data
    */
    void gpsCallback(const sensor_msgs::NavSatFix fix)
    {
        if (!setStartingPosition && !testing)
        {
            string utmZone = "";
            double northing = 0.0;
            double easting = 0.0;           

            gps_common::LLtoUTM((-1 * fix.latitude), fix.longitude, northing, easting, utmZone); //Invert latitude as we are in the southern half

            gpsUTMOrigin.x = northing;
            gpsUTMOrigin.y = easting;

            setStartingPosition = true;

            if (setStartingOrientation)
            {
                //Read in config file
                readConfigFile();

                //Push out the first goal
                loadNewGoal();

                goalAttemptLimit++;
            }
        }
    }

    /**
      Callback for the IMU
      @param imu The IMU data
    */
    void imuCallback(const sensor_msgs::Imu imu)
    {
        if (!setStartingOrientation)
        {
            angleOffNorth = (180 * imu.orientation.z) - 1.617; //Minus 1.617 because magnetic and true north are not the same. This will need changing for the competition

            setStartingOrientation = true;

            if (setStartingPosition)
            {
                //Read in config file
                readConfigFile();

                //Push out the first goal
                loadNewGoal();

                goalAttemptLimit++;
            }
        }
    }

    /**
      Callback for the hector_pose_estimation position estimation
      @param pose The actual pose data
    */
    void hlPoseCallback(const geometry_msgs::PoseStamped pose)
    {
        //First convert the PoseStamped into Odometry and publish for use by Movebase
        nav_msgs::Odometry toPublish;

        toPublish.child_frame_id = "base_link";
        toPublish.header = pose.header;
        toPublish.pose.pose = pose.pose;

        hlOdomPublisher.publish(toPublish);

        if (setStartingPosition)
        {
            currentCallbackCount++;

            //Update the distance to all the goals
            for (list<Goal>::iterator it = coordsList.begin(); it != coordsList.end(); it++)
            {
                double deltaX = (*it).x - pose.pose.position.x;
                double deltaY = (*it).y - pose.pose.position.y;

                (*it).distanceFromRobot = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
            }

            //If we have done enough position updates then try a goal update
            if (currentCallbackCount >= callbacksUntilGoalUpdate)
            {                
                ROS_INFO("\033[2;32mGoalProvider: Update goal?...\033[0m\n");
                publishStatus(AutonomousStatus::UPDATING, "Update Goal?");

                loadNewGoal();
                currentCallbackCount = 0;
            }
        }
        else if (testing && !setStartingPosition)
        {
            gpsUTMOrigin.x = pose.pose.position.x;
            gpsUTMOrigin.y = pose.pose.position.y;

            readConfigFile();
            loadNewGoal();

            goalAttemptCounter++;

            setStartingPosition = true;
        }
    }

    /**
      Callback for the move_base feedback
      @param feedback The actual feedback data
    */
    void movebaseFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback feedback)
    {
        //Movebase feedback is based off of wheel odometry and hence is less accurate than using Hector Localization.
        //We could replace the odometry with hector localization but this would require conversion from PoseStamped to Odometry
        //currentCallbackCount++;
    }

    /**
      Callback for the move_base status
      @param movebase_status The actual status data
    */
    void movebaseStatusCallback(const actionlib_msgs::GoalStatusArray movebase_status)
    {
        if (setStartingPosition)
        {
            if (movebase_status.status_list.size() > 0 && coordsList.size() > 0)
            {
                actionlib_msgs::GoalStatus s = movebase_status.status_list.front();
                char buffer[200];

                switch (s.status)
                {
                    case actionlib_msgs::GoalStatus::PENDING:
                        ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) is pending\033[0m\n", goal.x, goal.y);

                        sprintf(buffer, "Goal (%f, %f) is pending", goal.x, goal.y);
                        publishStatus(AutonomousStatus::WAITING, buffer);
                        break;

                    case actionlib_msgs::GoalStatus::PREEMPTED:
                        ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) preempted\033[0m\n", goal.x, goal.y);

                        sprintf(buffer, "Goal (%f, %f) preempted", goal.x, goal.y);
                        publishStatus(AutonomousStatus::PREEMPTED, buffer);

                        goalAcceptanceCounter++;
                        break;

                    case actionlib_msgs::GoalStatus::ACTIVE:
                        ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) accepted\033[0m\n", goal.x, goal.y);

                        sprintf(buffer, "Goal (%f, %f) accepted", goal.x, goal.y);
                        publishStatus(AutonomousStatus::ACTIVE, buffer);

                        goalAcceptanceCounter++;
                        break;

                    case actionlib_msgs::GoalStatus::SUCCEEDED:
                        if (goalAcceptanceCounter > 2)
                        {
                            goalAcceptanceCounter = 0;
                            currentCallbackCount = 0;
                            ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) reached\033[0m\n", goal.x, goal.y);

                            sprintf(buffer, "Goal (%f, %f) reached", goal.x, goal.y);
                            publishStatus(AutonomousStatus::REACHED, buffer);

                            removeCurrentGoal();

                            if (coordsList.size() > 0)
                            {
                                loadNewGoal();
                            }
                        }

                        break;

                    case actionlib_msgs::GoalStatus::REJECTED:
                        ROS_INFO("\033[1;31mGoalProvider: Goal (%f, %f) rejected\033[0m\n", goal.x, goal.y);

                        sprintf(buffer, "Goal (%f, %f) rejected", goal.x, goal.y);
                        publishStatus(AutonomousStatus::REJECTED, buffer);

                        goalAttemptCounter++;
                        loadNewGoal();
                        break;

                    case actionlib_msgs::GoalStatus::ABORTED:
                        ROS_INFO("\033[1;31mGoalProvider: Goal (%f, %f) aborted\033[0m\n", goal.x, goal.y);

                        sprintf(buffer, "Goal (%f, %f) aborted", goal.x, goal.y);
                        publishStatus(AutonomousStatus::ABORTED, buffer);

                        goalAttemptCounter++;
                        loadNewGoal(); //Not sure if this will entirely work. This callback may happen more than once before a new goal is pushed out
                        break;
                }
            }
        }
    }

    /**
      Read in the GPSConfig file to get the list of UTM based Points we need to go to
    */
    void readConfigFile()
    {
        ifstream configFile("/home/wambot/fuerte_workspace/goal_provider/config/GPSCoords");

        if (!configFile.is_open())
        {
            ROS_ERROR("\033[1;31mGoalProvider: Could not load config file\033[0m\n");
            publishStatus(AutonomousStatus::ERROR, "Could not load config file");

        }
        else
        {
            string line;
            int lineNumber = 1;

            while(getline(configFile, line))
            {
                Goal p;

                double latitude = 0;
                double longitude = 0;
                string utmZone;

                istringstream iss(line);

                if (!(iss >> latitude >> longitude))
                {
                    ROS_ERROR("\033[1;31mGoalProvider: Could not read in GPS data on line %i\033[0m\n", lineNumber);

                    char buffer[200];
                    sprintf(buffer, "Could not read in the GPS data on %i", lineNumber);
                    publishStatus(AutonomousStatus::ERROR, buffer);
                    break;
                }

                if (!testing)
                {
                    latitude *= -1; //Invert latitude as we are in the southern half

                    gps_common::LLtoUTM(latitude, longitude, p.x, p.y, utmZone);

                    p.x -= gpsUTMOrigin.x;
                    p.y -= gpsUTMOrigin.y;

                    p.y *= -1; //Flip this so that it is the correct direction for the robot

                    p.distanceFromRobot = sqrt(pow((p.x), 2) + pow((p.y), 2));

                    if (angleOffNorth >= 0)
                    {
                        p.x = (p.x * cos(angleOffNorth*PI/180)) - (p.y * sin(angleOffNorth*PI/180));
                        p.y = (p.x * sin(angleOffNorth*PI/180)) + (p.y * cos(angleOffNorth*PI/180));
                    }
                    else
                    {
                        p.x = (p.x * cos(angleOffNorth*PI/180)) + (p.y * sin(angleOffNorth*PI/180));
                        p.y = -(p.x * sin(angleOffNorth*PI/180)) + (p.y * cos(angleOffNorth*PI/180));
                    }

                    ROS_INFO("\033[2;32mGoalProvider: Read in and converted GPS to UTM: (x: %f, y: %f, distance: %f)\033[0m\n", p.x, p.y, p.distanceFromRobot);

                    char buffer[200];
                    sprintf(buffer, "Read in and converted GPS to UTM: (x: %f, y: %f, distance: %f)", p.x, p.y, p.distanceFromRobot);
                    publishStatus(AutonomousStatus::INFO, buffer);

                    coordsList.push_front(p);
                }
                else
                {
                    p.x = latitude;
                    p.y = longitude;

                    p.distanceFromRobot = sqrt(pow((p.x - gpsUTMOrigin.x), 2) + pow((p.y - gpsUTMOrigin.y), 2));

                    ROS_INFO("\033[2;32mGoalProvider: Read in local coordinates: (x: %f, y: %f, distance: %f)\033[0m\n", p.x, p.y, p.distanceFromRobot);

                    char buffer[200];
                    sprintf(buffer, "Read in local coordinates: (x: %f, y: %f, distance: %f)", p.x, p.y, p.distanceFromRobot);
                    publishStatus(AutonomousStatus::INFO, buffer);

                    coordsList.push_front(p);
                }

                lineNumber++;
            }
        }

        configFile.close();
    }

    /**
      Removes the current goal. Used when the goal is achieved, broken etc...
    */
    void removeCurrentGoal()
    {
        if (coordsList.size() > 0)
        {
            coordsList.pop_front();
        }
    }

    /**
      Loads a new movement goal. The closest GPS goal is chosen as the desired goal
    */
    void loadNewGoal()
    {
        if (coordsList.size() > 0)
        {
            coordsList.sort(); //Sort the goal points in case they have changed in order

            Goal newGoal = coordsList.front();

            char buffer[200];

            if (!sentFirstGoal)
            {
                sentFirstGoal = true;

                ROS_INFO("\033[2;32mGoalProvider: Sending first goal(%f, %f)\033[0m\n", newGoal.x, newGoal.y);
                goal = newGoal;

                sprintf(buffer, "Sending first goal(%f, %f)", newGoal.x, newGoal.y);
                publishStatus(AutonomousStatus::INFO, buffer);

                goalAttemptCounter = 1;
                sendGoal(); //Send the new one
            }
            else if (newGoal == goal && (goalAttemptCounter >= goalAttemptLimit)) //If the goal is the same but the goal limit has been exceeded
            {
                ROS_INFO("\033[2;33mGoal:Provider: The goal (%f, %f) timed out\033[0m\n", goal.x, goal.y);
                removeCurrentGoal(); //delete the current goal and get a new one

                sprintf(buffer, "The goal (%f, %f) timed out", goal.x, goal.y);
                publishStatus(AutonomousStatus::INFO, buffer);

                goal = coordsList.front();

                goalAttemptCounter = 1;
                sendGoal(); //Send the new one
            }
            else if (newGoal != goal) //If the goal is different to the current one then send it
            {
                ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) replaced goal (%f, %f)\033[0m\n", newGoal.x, newGoal.y, goal.x, goal.y);
                goal = newGoal;

                sprintf(buffer, "Goal (%f, %f) replaced goal (%f, %f)", newGoal.x, newGoal.y, goal.x, goal.y);
                publishStatus(AutonomousStatus::INFO, buffer);

                goalAttemptCounter = 1;
                sendGoal(); //Send the new one
            }//If the closest goal is the same as the current one and the attempt limit is not exceeded then continue onwards
        }
        else
        { //Head home
            goal.distanceFromRobot = sqrt(pow(goal.x, 2) + pow(goal.y, 2));
            goal.x = 0;
            goal.y = 0;

            sendGoal();
        }
    }

    /**
      Sends the current goal to the robot
    */
    void sendGoal()
    {
        move_base_msgs::MoveBaseGoal target_goal;

        target_goal.target_pose.header.frame_id = "odom";
        target_goal.target_pose.header.stamp = ros::Time::now();

        target_goal.target_pose.pose.position.x = goal.x;
        target_goal.target_pose.pose.position.y = goal.y;
        target_goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("\033[2;33mGoalProvider: Attempting to send goal (%f, %f). Distance %f m\033[0m\n", goal.x, goal.y, goal.distanceFromRobot);

        char buffer[200];
        sprintf(buffer, "Attempting to send goal (%f, %f). Distance %f m", goal.x, goal.y, goal.distanceFromRobot);
        publishStatus(AutonomousStatus::WAITING, buffer);

        mb.sendGoal(target_goal);
    }

    /**
      Encapsulates all status updates made by this node

      @param currentStatus The status we wish to publish
      @param message Any extra message information we wish to send
    */
    void publishStatus(uint currentStatus, string message)
    {
        status.status = currentStatus;
        status.goal.x = goal.x;
        status.goal.y = goal.y;
        status.goal.z = goal.distanceFromRobot;

        std_msgs::String text;
        text.data = message;
        status.text = text;

        autonomousStatusPublisher.publish(status);
    }
};

/**
  Entry point for the application
*/
int main (int argc, char** argv)
{
    ros::init(argc, argv, "goal_provider_node");

    GoalProvider node;

    ros::spin();

    return 0;
}

/**
  Function to write to serial for communication with an Arduino (hard method)
  @param value The value to send
*/
void writeToSerialHardMethod(int value)
{
    int fileDescription = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);

    if (fileDescription == -1)
    {
        ROS_ERROR("\033[1;31mGoalProvider: Could not open serial connection\033[0m\n");
    }
    else
    {
        fcntl(fileDescription, F_SETFL, 0);

        //http://www.cplusplus.com/forum/beginner/6914/
        struct termios port_settings;      // structure to store the port settings in

        cfsetispeed(&port_settings, B115200);    // set baud rates
        cfsetospeed(&port_settings, B115200);

        port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
        port_settings.c_cflag &= ~CSTOPB;
        port_settings.c_cflag &= ~CSIZE;
        port_settings.c_cflag |= CS8;

        tcsetattr(fileDescription, TCSANOW, &port_settings);    // apply the settings to the port

        char n;
        fd_set rdfs;
        struct timeval timeout;

        // initialise the timeout structure
        timeout.tv_sec = 10; // ten second timeout
        timeout.tv_usec = 0;

        char buffer[10];
        sprintf(buffer, "%i\n", value);

        write(fileDescription, buffer, 2);
        printf("Wrote the bytes. \n");

        // do the select
        n = select(fileDescription + 1, &rdfs, NULL, NULL, &timeout);

        // check if an error has occured
        if(n < 0)
        {
            ROS_ERROR("select failed\n");
        }
        else if (n == 0)
        {
            ROS_ERROR("Timeout!");
        }
        else
        {
            ROS_INFO("\nBytes detected on the port!\n");
        }

        close(fileDescription);
    }
}

/**
  Function to write to Arduino using serial port (easy method)
  @param value The value to write
*/
void writeToSerialEasyMethod(int value)
{
    fstream serial;

    serial.open("/dev/ttyS0");

    if (serial.is_open())
    {
        serial << value;
    }
    else
    {
        ROS_ERROR("Could not open serial port");
    }

    serial.close();
}

