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

namespace GoalProvider
{
    /**
        The GoalProvider class.

        This class provides goal data to the move_base navigation stack. A typical goal is simply a GPS position

        @author Enda McCauley
    */
    class Goal_Provider
    {
    public:
        Goal_Provider() : mb("move_base", true)
        {
            NodeHandle node;
            NodeHandle privateNode("~");

            /*
            while(!mb.waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("\033[2;33mWaiting for the move_base action server to come up\033[0m\n");
            }
            */

            setOrigin = false;
            startTime = ros::Time::now();
            currentCallbackCount = 0;
            goalAttemptCounter = 0;

            ROS_INFO("\033[2;32mGoalProvider: Initialized at time: %d.%d\033[0m\n", startTime.sec, startTime.nsec);

            privateNode.param("callbacks_till_goal_update", callbacksUntilGoalUpdate, 1000);
            privateNode.param("competition_length", competitionTimeLimit, 100);
            privateNode.param("time_cutoff", cutoffTime, 80);
            privateNode.param("goal_attempt_limit", goalAttemptLimit, 5);
            privateNode.param("testing", testing, false);

            ROS_INFO("Callbacks, %d", callbacksUntilGoalUpdate);

            gpsSubscriber = node.subscribe("/GPSOfSomeSort", 1000, &Goal_Provider::gpsCallback, this);
            hlPoseSubscriber = node.subscribe("localization_pose", 1000, &Goal_Provider::hlPoseCallback, this);
            movebaseFeedbackSubscriber = node.subscribe("move_base/feedback", 1000, &Goal_Provider::movebaseFeedbackCallback, this);
            movebaseStatusSubscriber = node.subscribe("move_base/status", 1000, &Goal_Provider::movebaseStatusCallback, this);

            autonomousStatusPublisher = privateNode.advertise<goal_provider::AutonomousStatus>("/status", 1000);
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
                pushNewGoal();

                goalAttemptLimit++;
            }
        }

        /**
          Callback for the hector_pose_estimation position estimation
          @param pose The actual pose data
        */
        void hlPoseCallback(const geometry_msgs::Pose pose)
        {
            if (setOrigin)
            {
                currentCallbackCount++;

                for (list<Goal>::iterator it = coordsList.begin(); it != coordsList.end(); it++)
                {
                    double deltaX = (*it).x - (gpsUTMOrigin.x + pose.position.x);
                    double deltaY = (*it).y - (gpsUTMOrigin.y + pose.position.y);

                    (*it).distanceFromRobot = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
                }

                if (currentCallbackCount >= callbacksUntilGoalUpdate)
                {
                    ROS_INFO("\033[2;32mGoalProvider: Update goal?...\033[0m\n");

                    pushNewGoal();
                    currentCallbackCount = 0;
                }
            }
            else if (testing && !setOrigin)
            {
                gpsUTMOrigin.x = pose.position.x;
                gpsUTMOrigin.y = pose.position.y;

                readConfigFile();
                pushNewGoal();

                goalAttemptLimit++;

                setOrigin = true;
            }
        }

        /**
          Callback for the move_base feedback
          @param feedback The actual feedback data
        */
        void movebaseFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback feedback)
        {
            //Movebase feedback is based off of wheel odometry and hence is less accurate than using Hector Localization
            currentCallbackCount++;
        }

        /**
          Callback for the move_base status
          @param movebase_status The actual status data
        */
        void movebaseStatusCallback(const actionlib_msgs::GoalStatusArray movebase_status)
        {
            const actionlib_msgs::GoalStatus &s = movebase_status.status_list.front();

            switch (s.status)
            {
                case actionlib_msgs::GoalStatus::PENDING:
                    ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) is pending\033[0m\n", goal.x, goal.y);
                    publishStatus(AutonomousStatus::GOAL_WAITING, "");
                    break;

                case actionlib_msgs::GoalStatus::ACTIVE:
                    ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) accepted\033[0m\n", goal.x, goal.y);
                    publishStatus(AutonomousStatus::GOAL_ACTIVE, "");
                    break;

                case actionlib_msgs::GoalStatus::SUCCEEDED:
                    ROS_INFO("\033[2;32mGoalProvider: Goal (%f, %f) reached\033[0m\n", goal.x, goal.y);
                    publishStatus(AutonomousStatus::GOAL_REACHED, "");

                    removeCurrentGoal();
                    pushNewGoal();
                    break;

                case actionlib_msgs::GoalStatus::REJECTED:
                    ROS_ERROR("\033[1;31mGoalProvider: Goal (%f, %f) rejected\033[0m\n", goal.x, goal.y);
                    publishStatus(AutonomousStatus::GOAL_REJECTED, "");

                    goalAttemptLimit++;
                    pushNewGoal();
                    break;

                case actionlib_msgs::GoalStatus::ABORTED:
                    ROS_ERROR("\033[1;31mGoalProvider: Goal (%f, %f) aborted\033[0m\n", goal.x, goal.y);
                    publishStatus(AutonomousStatus::GOAL_ABORTED, "");

                    goalAttemptLimit++;
                    pushNewGoal(); //Not sure if this will entirely work. This callback may happen more than once before a new goal is pushed out
                    break;
            }
        }

        /**
          Read in the GPSConfig file to get the list of UTM based Points we need to go to
        */
        void readConfigFile()
        {
            ifstream configFile("/home/enda/fuerte_workspace/sandbox/goal_provider/config/GPSCoords");

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

                    latitude *= -1; //Invert latitude as we are in the southern half

                    if (!testing)
                    {
                        gps_common::LLtoUTM(latitude, longitude, p.y, p.x, utmZone);
                    }

                    p.distanceFromRobot = sqrt(pow((p.x - gpsUTMOrigin.x), 2) + pow((p.y - gpsUTMOrigin.y), 2));

                    ROS_INFO("\033[2;32mGoalProvider: Read in and converted GPS to UTM: (x: %f, y: %f, distance: %f)\033[0m\n", p.x, p.y, p.distanceFromRobot);

                    coordsList.push_front(p);

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
            coordsList.pop_front();
        }

        /**
          Pushes a new movement goal to move_base. The closest GPS goal is chosen as the desired goal
        */
        void pushNewGoal()
        {
            coordsList.sort(); //Sort the goal points in case they have changed in order

            Goal newGoal = coordsList.front();

            if (newGoal == goal && goalAttemptLimit >= goalAttemptCounter)
            {
                removeCurrentGoal();

                goal = coordsList.front();
            }
            else if (newGoal != goal)
            {
                goal = newGoal;

                goalAttemptLimit = 1;
            }

            move_base_msgs::MoveBaseGoal target_goal;

            target_goal.target_pose.header.frame_id = "base_link";
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
        }
    };
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "goal_provider_node");

    GoalProvider::Goal_Provider node;

    ros::spin();

    return 0;
}

