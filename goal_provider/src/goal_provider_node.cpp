#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
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

        while(!mb.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("\033[2;33mWaiting for the move_base action server to come up\033[0m\n");
        }

        setOrigin = false;
        sentFirstGoal = false;
        startTime = ros::Time::now();
        currentCallbackCount = 0;
        goalAttemptCounter = 0;
        goalAcceptanceCounter = 0;

        ROS_INFO("\033[2;32mGoalProvider: Initialized at time: %d.%d\033[0m\n", startTime.sec, startTime.nsec);

        privateNode.param("callbacks_till_goal_update", callbacksUntilGoalUpdate, 1000);
        privateNode.param("competition_length", competitionTimeLimit, 100);
        privateNode.param("time_cutoff", cutoffTime, 80);
        privateNode.param("goal_attempt_limit", goalAttemptLimit, 5);
        privateNode.param("testing", testing, false);

        ROS_INFO("\033[2;32mCallbacks until goal update set to: %d\033[0m\n", callbacksUntilGoalUpdate);
        ROS_INFO("\033[2;32mCompetition Length set to: %d\033[0m\n", competitionTimeLimit);
        ROS_INFO("\033[2;32mTime cutoff set to: %d\033[0m\n", cutoffTime);
        ROS_INFO("\033[2;32mGoal attempt limit set to: %d\033[0m\n", goalAttemptLimit);
        ROS_INFO("\033[2;32mTesting set to: %d\033[0m\n", (testing) ? 1 : 0);

        gpsSubscriber = node.subscribe("/GPSOfSomeSort", 1000, &GoalProvider::gpsCallback, this);
        hlPoseSubscriber = node.subscribe("localization_pose", 1000, &GoalProvider::hlPoseCallback, this);
        movebaseFeedbackSubscriber = node.subscribe("move_base/feedback", 1000, &GoalProvider::movebaseFeedbackCallback, this);
        movebaseStatusSubscriber = node.subscribe("move_base/status", 1000, &GoalProvider::movebaseStatusCallback, this);

        autonomousStatusPublisher = privateNode.advertise<goal_provider::AutonomousStatus>("/status", 1000);
        hlOdomPublisher = privateNode.advertise<nav_msgs::Odometry>("/odom", 1000);
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
      Field to store the number of times we have set out origin
    */
    bool setOrigin;

    /**
      Field to store if we are testing without GPS or not
    */
    bool testing;

    /**
      Field to store whether or not we have sent the first goal
    */
    bool sentFirstGoal;

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
    void gpsCallback(const nav_msgs::Odometry& odom)
    {
        if (!setOrigin && !testing)
        {
            gpsUTMOrigin.x = odom.pose.pose.position.x;
            gpsUTMOrigin.y = odom.pose.pose.position.y;

            setOrigin = true;

            //Read in config file
            readConfigFile();

            //Push out the first goal
            loadNewGoal();

            goalAttemptLimit++;
        }
    }

    /**
      Callback for the hector_pose_estimation position estimation
      @param pose The actual pose data
    */
    void hlPoseCallback(const geometry_msgs::PoseStamped pose)
    {
        if (setOrigin)
        {
            currentCallbackCount++;

            for (list<Goal>::iterator it = coordsList.begin(); it != coordsList.end(); it++)
            {
                double deltaX = (*it).x - (gpsUTMOrigin.x + pose.pose.position.x);
                double deltaY = (*it).y - (gpsUTMOrigin.y + pose.pose.position.y);

                (*it).distanceFromRobot = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
            }

            if (currentCallbackCount >= callbacksUntilGoalUpdate)
            {
                ROS_INFO("\033[2;32mGoalProvider: Update goal?...\033[0m\n");

                loadNewGoal();
                currentCallbackCount = 0;
            }
        }
        else if (testing && !setOrigin)
        {
            gpsUTMOrigin.x = pose.pose.position.x;
            gpsUTMOrigin.y = pose.pose.position.y;

            readConfigFile();
            loadNewGoal();

            goalAttemptCounter++;

            setOrigin = true;
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
        if (setOrigin)
        {
            if (movebase_status.status_list.size() > 0)
            {
                actionlib_msgs::GoalStatus s = movebase_status.status_list.front();

                switch (s.status)
                {
                    case actionlib_msgs::GoalStatus::PENDING:
                        ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) is pending\033[0m\n", goal.x, goal.y);
                        publishStatus(AutonomousStatus::GOAL_WAITING, "");
                        break;

                    case actionlib_msgs::GoalStatus::PREEMPTED:
                        ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) preempted\033[0m\n", goal.x, goal.y);
                        publishStatus(AutonomousStatus::GOAL_PREEMPTED, "");

                        goalAcceptanceCounter++;
                        break;

                    case actionlib_msgs::GoalStatus::ACTIVE:
                        ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) accepted\033[0m\n", goal.x, goal.y);
                        publishStatus(AutonomousStatus::GOAL_ACTIVE, "");

                        goalAcceptanceCounter++;
                        break;

                    case actionlib_msgs::GoalStatus::SUCCEEDED:
                        if (goalAcceptanceCounter > 2)
                        {
                            goalAcceptanceCounter = 0;
                            ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) reached\033[0m\n", goal.x, goal.y);
                            publishStatus(AutonomousStatus::GOAL_REACHED, "");

                            removeCurrentGoal();
                            loadNewGoal();
                        }

                        break;

                    case actionlib_msgs::GoalStatus::REJECTED:
                        ROS_ERROR("\033[1;31mGoalProvider: Goal (%f, %f) rejected\033[0m\n", goal.x, goal.y);
                        publishStatus(AutonomousStatus::GOAL_REJECTED, "");

                        goalAttemptCounter++;
                        loadNewGoal();
                        break;

                    case actionlib_msgs::GoalStatus::ABORTED:
                        ROS_ERROR("\033[1;31mGoalProvider: Goal (%f, %f) aborted\033[0m\n", goal.x, goal.y);
                        publishStatus(AutonomousStatus::GOAL_ABORTED, "");

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
                    break;
                }

                if (!testing)
                {
                    latitude *= -1; //Invert latitude as we are in the southern half

                    gps_common::LLtoUTM(latitude, longitude, p.y, p.x, utmZone);

                    p.distanceFromRobot = sqrt(pow((p.x - gpsUTMOrigin.x), 2) + pow((p.y - gpsUTMOrigin.y), 2));

                    ROS_INFO("\033[2;32mGoalProvider: Read in and converted GPS to UTM: (x: %f, y: %f, distance: %f)\033[0m\n", p.x, p.y, p.distanceFromRobot);

                    coordsList.push_front(p);
                }
                else
                {
                    p.x = latitude;
                    p.y = longitude;

                    p.distanceFromRobot = sqrt(pow((p.x - gpsUTMOrigin.x), 2) + pow((p.y - gpsUTMOrigin.y), 2));

                    ROS_INFO("\033[2;32mGoalProvider: Read in local coordinates: (x: %f, y: %f, distance: %f)\033[0m\n", p.x, p.y, p.distanceFromRobot);

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

            if (newGoal == goal && (goalAttemptCounter >= goalAttemptLimit)) //If the goal is the same but the goal limit has been exceeded
            {
                removeCurrentGoal(); //delete the current goal and get a new one

                goal = coordsList.front();

                if (sentFirstGoal)
                {
                    mb.cancelGoal();
                }

                goalAttemptCounter = 1;
                sendGoal(); //Send the new one
            }
            else if (newGoal != goal) //If the goal is different to the current one then send it
            {
                goal = newGoal;

                if (sentFirstGoal)
                {
                    mb.cancelGoal();
                }

                goalAttemptCounter = 1;
                sendGoal(); //Send the new one
            }//If the closest goal is the same as the current one and the attempt limit is not exceeded then continue onwards
        }
        else
        { //Head home
            goal.distanceFromRobot = sqrt(pow(goal.x, 2) + pow(goal.y, 2));
            goal.x = 0;
            goal.y = 0;

            if (sentFirstGoal)
            {
                mb.cancelGoal();
            }

            sendGoal();
        }

        if (!sentFirstGoal)
        {
            sentFirstGoal = true;
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

        publishStatus(AutonomousStatus::GOAL_WAITING, "");

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

