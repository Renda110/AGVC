#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <p2os_driver/MotorState.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

using namespace ros;
using namespace std;

/**
  Class to handle initialisation of the software necessary for the AGVC competition
  @author Enda McCauley
*/
class AGVC_Coordinator
{
public:
    /**
      Constructs a new AGVC_Coordinator object
    */
    AGVC_Coordinator()
    {
        NodeHandle node;
        NodeHandle privateNode("~");

        privateNode.param("joystick", usingJoystick, false);
        ROS_INFO("\033[2;32mAGVC Coordinator: Using Joystick set to: %d\033[0m\n", (usingJoystick) ? 1 : 0);

        motorPublisher = node.advertise<p2os_driver::MotorState>("cmd_motor_state", 1000);

        fullActive = false;
        partialActive = false;
        motorsActive = false;

        if (!usingJoystick)
        {
            waitForCommand();
        }
        else
        {
            joySubscriber = node.subscribe("/joy", 1000, &AGVC_Coordinator::joyCallback, this);
            printJoystickHelp();
        }
    }

private:
    /**
      Field to store the subscriber for joystick message
    */
    Subscriber joySubscriber;

    /**
      Field to store the publisher to enable/disable the robot motors
    */
    Publisher motorPublisher;

    /**
      Field to store whether or not the joystick is being used
    */
    bool usingJoystick;

    /**
      Field to store whether or not the full system is active (full autonomous operation)
    */
    bool fullActive;

    /**
      Field to store whether or not the system is only partially active (joystick and bag file)
    */
    bool partialActive;

    /**
      Field to store whether or not the motors are active
    */
    bool motorsActive;

    /**
      Callback for joystick input
      @param joy The joystick input
    */
    void joyCallback(const sensor_msgs::Joy joy)
    {
        //Joystick codes
        // buttons[0] = 1 -> 1
        // buttons[1] = 1 -> 2
        // buttons[2] = 1 -> 3
        // buttons[3] = 1 -> 4
        // buttons[9] = 1 -> 10

        if (joy.buttons[0] == 1) //Button 1: Start full mode
        {
            if (!fullActive && !partialActive)
            {
                startFull(false);
            }
            else
            {
                cout << "\033[2;31mAGVC_Coordinator: Another mode is active\033[0m" << endl;
            }
        }
        else if (joy.buttons[1] == 1) //Button 2: Stop whichever mode is active
        {
            if (fullActive)
            {
                stopFull();
            }
            else if (partialActive)
            {
                stopPartial();
            }
            else
            {
                cout << "\033[2;31mAGVC_Coordinator: Nothing to stop\033[0m" << endl;
            }
        }
        else if (joy.buttons[2] == 1) //Button 3: Start partial mode
        {
            if (!partialActive && !fullActive)
            {
                startPartial(false);
            }
            else
            {
                cout << "\033[2;31mAGVC_Coordinator: Another mode is active\033[0m" << endl;
            }
        }
        else if (joy.buttons[3] == 1) //Button 4: Switch motor states
        {
            if (fullActive || partialActive)
            {
                toggleMotorState(false);
            }
            else
            {
                cout << "\033[2;31mAGVC_Coordinator: No mode is active\033[0m" << endl;
            }
        }
        else if (joy.buttons[9] == 1) //Button 10: Turn everything off including this node
        {
            if (fullActive)
            {
                stopFull();
            }
            else if (partialActive)
            {
                stopPartial();
            }

            totalShutdown();

            cout << "\033[2;31mExiting... \033[0m" << endl;
            exit(0);
        }
    }

    /**
      The function that does all the work. It waits for keyboard input and then depending on the input carries out a particular action
    */
    void waitForCommand()
    {
        string input = "";
        cout << "\033[2;36mAGVC_Coordinator> \033[2;32mEnter the desired operation or help to get help\033[0m" << endl;
        cout << "\033[2;36mAGVC_Coordinator> \033[0m";
        getline(cin, input);

        while (true)
        {
            if (input == "help" || input == "Help")
            {
                printKeyboardHelp();
            }
            else if (input == "full" || input == "Full")
            {
                startFull(true);
            }
            else if (input == "record" || input == "Record")
            {
                startPartial(true);
            }
            else if (input == "stop" || input == "Stop")
            {
                if (fullActive)
                {
                    stopFull();
                }
                else if (partialActive)
                {
                    stopPartial();
                }
            }
            else if (input == "q" || input == "quit" || input == "Quit" || input == "exit" || input == "Exit")
            {
                if (fullActive)
                {
                    stopFull();
                }
                else if (partialActive)
                {
                    stopPartial();
                }

                totalShutdown();

                cout << "\033[1;31mAGVC Coordinator is exiting... \033[0m" << endl;
                break;
            }
            else
            {
                cout << "\033[2;31mUnknown command. Enter help to get help\033[0m" << endl;
            }

            cout << "\033[2;36mAGVC_Coordinator> \033[0m";
            getline(cin, input);
        }

        exit(0);
    }

    /**
      Prints keyboard related help text for the user
    */
    void printKeyboardHelp()
    {
        cout << "\n\033[2;32m********************************************************************************************************************\033[0m" << endl;
        cout << "\033[2;32mAvailable commands:\033[0m" << endl;
        cout << "\033[2;32m\t full or Full:\033[0m" << endl;
        cout << "\033[2;32m\t\t Use this for full autonomous mode and goal planning\033[0m" << endl;
        cout << "\033[2;32m\t record or Record: \033[0m" << endl;
        cout << "\033[2;32m\t\t Use this to start everything except for the path planner and the goal provider.\033[0m" << endl;
        cout << "\033[2;32m\t\t This mode also allows full joystick input and a bag file is started to record everything\033[0m" << endl;
        cout << "\033[2;32m\t stop or Stop:\033[0m" << endl;
        cout << "\033[2;32m\t\t Stops all rosnodes except for this one\033[0m" << endl;
        cout << "\033[2;32m\t q, quit, Quit, exit or Exit: \033[0m" << endl;
        cout << "\033[2;32m\t\t Enter any of these to stop this node as well as all other nodes\033[0m" << endl;
        cout << "\033[2;32m********************************************************************************************************************\033[0m" << endl;
    }

    /**
      Prints joystick related help text for the user
    */
    void printJoystickHelp()
    {
        cout << "\n\033[2;32m********************************************************************************************************************\033[0m" << endl;
        cout << "\033[2;32mAvailable commands:\033[0m" << endl;
        cout << "\033[2;32m\t 1\033[0m" << endl;
        cout << "\033[2;32m\t\t Use this for full autonomous mode and goal planning\033[0m" << endl;
        cout << "\033[2;32m\t 3 \033[0m" << endl;
        cout << "\033[2;32m\t\t Use this to start everything except for the path planner and the goal provider.\033[0m" << endl;
        cout << "\033[2;32m\t\t This mode also allows full joystick input and a bag file is started to record everything\033[0m" << endl;
        cout << "\033[2;32m\t 2\033[0m" << endl;
        cout << "\033[2;32m\t\t Stops all rosnodes except for this one\033[0m" << endl;
        cout << "\033[2;32m\t 4\033[0m" << endl;
        cout << "\033[2;32m\t\t Turns the motors on or off\033[0m" << endl;
        cout << "\033[2;32m\t 10 \033[0m" << endl;
        cout << "\033[2;32m\t\t Enter any of these to stop this node as well as all other nodes\033[0m" << endl;
        cout << "\033[2;32m********************************************************************************************************************\033[0m" << endl;
    }

    /**
      Toggles the motor state between on or off
      @param shutdown Whether or not the system is being shutdown
    */
    void toggleMotorState(bool shutdown)
    {
        p2os_driver::MotorState ms;

        if (!shutdown)
        {
            if (motorsActive)
            {
                ms.state = 0;
                cout << "\033[2;32mAGVC_Coordinator: Motors disabled\033[0m" << endl;
                motorsActive = !motorsActive;
            }
            else
            {
                ms.state = 1;
                cout << "\033[2;32mAGVC_Coordinator: Motors enabled\033[0m" << endl;
                motorsActive = !motorsActive;
            }
        }
        else
        {
            ms.state = 0;
            cout << "\033[2;32mAGVC_Coordinator: Motors disabled\033[0m" << endl;
            motorsActive = false;
        }

        motorPublisher.publish(ms);
    }

    /**
      Launches the necessary nodes for the full autonomous mode
      @param motors Whether or not we want the motors enabled as well
    */
    void startFull(bool motors)
    {
        system ("roslaunch agvc_coordinator xsens_driver.launch &" ); //Ideally this will be one launch file
        system ("roslaunch agvc_coordinator AGVCFull.launch &");

        fullActive = true;

        if (motors)
        {
            toggleMotorState(false);
        }

        cout << "\033[2;32mAGVC_Coordinator: Full autonomous mode activated\033[0m" << endl;
    }

    /**
      Stops the full autonomous mode
    */
    void stopFull()
    {
        cout << "\033[2;32mAGVC_Coordinator: Stopping full mode\033[0m" << endl;
        toggleMotorState(true);

        system ("rosnode list > nodes");

        killNodes();

        fullActive = false;
    }

    /**
      Launches the necessary nodes for the partial mode
      @param Whether or not we want the motors enabled as well
    */
    void startPartial(bool motors)
    {
        system ("roslaunch agvc_coordinator AGVCPartial.launch &");
        //system ("roslaunch hector_pose_estimation hector_pose_estimation.launch &");

        partialActive = true;

        if (motors)
        {
            toggleMotorState(false);
        }

        cout << "\033[2;32mAGVC_Coordinator: Partial mode activated\033[0m" << endl;
    }

    /**
      Stops the partial mode
    */
    void stopPartial()
    {
        cout << "\033[2;32mAGVC_Coordinator: Stopping partial mode\033[0m" << endl;

        toggleMotorState(true);

        system ("rosnode list > nodes");

        killNodes();

        partialActive = false;
    }

    /**
      Kills all the nodes found in the "nodes" file
    */
    void killNodes()
    {
        ifstream nodeFile("nodes");

        if (!nodeFile.is_open())
        {
            cout << "\033[1;31mGoalProvider: Could not load nodes file\033[0m\n" << endl;
        }
        else
        {
            string line;

            while(getline(nodeFile, line))
            {
                if (line != "/agvc_coordinator" && line != "/rosout" && line != "/rosout_agg" && line != "/joy_node")
                {
                    char buffer[100];
                    sprintf(buffer, "rosnode kill %s", line.c_str());
                    system (buffer);
                }
            }
        }

        nodeFile.close();
        system ("rm nodes &");
    }

    /**
      Shuts down any remaining nodes (we can't actually kill everything - rosout will remain running)
    */
    void totalShutdown()
    {
        system ("rosnode kill joy_node");
    }
};

/**
  Entry point for the application
*/
int main (int argc, char** argv)
{
    ros::init(argc, argv, "agvc_coordinator");

    AGVC_Coordinator node;

    ros::spin();

    return 0;
}
