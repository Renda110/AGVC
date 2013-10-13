#include <stdlib.h>
#include <stdio.h>
#include <string.h>

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

        joySubscriber = node.subscribe("/joy", 1000, &AGVC_Coordinator::joyCallback, this);

        waitForCommand();
    }

private:
    /**
      Field to store the subscriber for joystick message
    */
    Subscriber joySubscriber;

    /**
      Callback for joystick input
      @param joy The joystick input
    */
    void joyCallback(const sensor_msgs::Joy joy)
    {
        //Do some stuff here maybe??
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
                printHelp();
            }
            else if (input == "full" || input == "Full")
            {
                system ("roslaunch hector_pose_estimation hector_pose_estimation.launch"); //Ideally this will be one launch file
            }
            else if (input == "record" || input == "Record")
            {

            }
            else if (input == "q" || input == "quit" || input == "Quit" || input == "exit" || input == "Exit")
            {
                cout << "\033[2;31mExiting... \033[0m" << endl;
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
      Prints help text for the user
    */
    void printHelp()
    {
        cout << "\033[2;32m********************************************************************************************************************\033[0m" << endl;
        cout << "\033[2;32mAvailable commands:\033[0m" << endl;
        cout << "\033[2;32m\t full or Full:\033[0m" << endl;
        cout << "\033[2;32m\t\t Use this for full autonomous mode and goal planning\033[0m" << endl;
        cout << "\033[2;32m\t record or Record: \033[0m" << endl;
        cout << "\033[2;32m\t\t Use this to start everything except for the path planner and the goal provider.\033[0m" << endl;
        cout << "\033[2;32m\t\t This mode also allows full joystick input and a bag file is started to record everything\033[0m" << endl;
        cout << "\033[2;32m\t q, quit, Quit, exit or Exit: \033[0m" << endl;
        cout << "\033[2;32m\t\t Enter any of these to effectively stop this node\033[0m" << endl;
        cout << "\033[2;32m********************************************************************************************************************\033[0m" << endl;
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
