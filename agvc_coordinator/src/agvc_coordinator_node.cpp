#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <p2os_driver/MotorState.h>
#include <sound_play/sound_play.h>
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
        privateNode.param("path", path, std::string("home/wambot/fuerte_workspace/"));

        ROS_INFO("\033[2;32mAGVC Coordinator: Using Joystick set to: %d\033[0m\n", (usingJoystick) ? 1 : 0);
        ROS_INFO("\033[2;32mAGVC Coordinator: Base path set to: %s\033[0m\n", path.c_str());

        motorPublisher = node.advertise<p2os_driver::MotorState>("cmd_motor_state", 1000);

        fullActive = false;
        partialActive = false;
        motorsActive = false;
        bagFileActive = false;

        setStartTime();

        sleep(10); //wait some time for the IMU and sound driver to become available. No other easy way to do this
        system ("roslaunch xsens_driver xsens_driver.launch &");

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
      Field to store the time this node started at
    */
    string startTime;

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
      Field to store the base path to this node
    */
    string path;

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
      Field to store whether or not a bag file is active
    */
    bool bagFileActive;

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
            startFull();
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
                cout << "\033[1;31mAGVC_Coordinator: Nothing to stop\033[0m" << endl;
                writeToLog("Nothing to stop");
            }
        }
        else if (joy.buttons[2] == 1) //Button 3: Start partial mode
        {
            startPartial();
        }
        else if (joy.buttons[3] == 1) //Button 4: Switch motor states
        {
            toggleMotorState(false);

        }
        else if (joy.buttons[7] == 1) //Shut the robot up
        {
            shutup();
        }
        else if (joy.buttons[8] == 1) //Button 9: Toggle a bag file on or off
        {
            if (bagFileActive)
            {
                stopBagfile();
            }
            else
            {
                startBagfile();
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
            writeToLog("Exiting...");
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
                startFull();
            }
            else if (input == "partial" || input == "Partial")
            {
                startPartial();
            }
            else if (input == "record" || input == "Record")
            {
                if (bagFileActive)
                {
                    stopBagfile();
                }
                else
                {
                    startBagfile();
                }
            }
            else if (input == "motors" || input == "Motors")
            {
                toggleMotorState(false);
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
                else
                {
                    cout << "\033[1;31mAGVC_Coordinator: Nothing to stop\033[0m" << endl;
                    writeToLog("Nothing to stop");
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
                writeToLog("Exiting...");
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
        cout << "\033[2;32m\t partial or Partial: \033[0m" << endl;
        cout << "\033[2;32m\t\t Use this to start everything except for the path planner and the goal provider.\033[0m" << endl;
        cout << "\033[2;32m\t\t This mode also allows full joystick input\033[0m" << endl;
        cout << "\033[2;32m\t record or Record: \033[0m" << endl;
        cout << "\033[2;32m\t\t Stops or starts a bag file recording.\033[0m" << endl;
        cout << "\033[2;32m\t motors or Motors\033[0m" << endl;
        cout << "\033[2;32m\t\t Enable or disable the motors\033[0m" << endl;
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
        cout << "\033[2;32m\t 9 \033[0m" << endl;
        cout << "\033[2;32m\t\t Press this to stop/start a bag file\033[0m" << endl;
        cout << "\033[2;32m\t 10 \033[0m" << endl;
        cout << "\033[2;32m\t\t Press this to stop this node as well as all other nodes\033[0m" << endl;
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
            if (fullActive || partialActive)
            {
                if (motorsActive)
                {
                    ms.state = 0;
                    cout << "\033[2;32mAGVC_Coordinator: Motors disabled\033[0m" << endl;
                    writeToLog("Motors disabled");
                    motorsActive = !motorsActive;

                    playSound("Motors disabled");
                }
                else
                {
                    ms.state = 1;
                    cout << "\033[2;32mAGVC_Coordinator: Motors enabled\033[0m" << endl;
                    writeToLog("Motors enabled");
                    motorsActive = !motorsActive;

                    playSound("Motors enabled");
                }
            }
            else
            {
                cout << "\033[1;31mAGVC_Coordinator: No mode is active\033[0m" << endl;
                writeToLog("No mode is active");

                playSound("No mode is active");
            }
        }
        else
        {
            ms.state = 0;
            cout << "\033[2;32mAGVC_Coordinator: Motors disabled\033[0m" << endl;
            writeToLog("Motors disabled");
            motorsActive = false;
        }

        motorPublisher.publish(ms);
    }

    /**
      Launches the necessary nodes for the full autonomous mode
    */
    void startFull()
    {
        if (!fullActive && !partialActive)
        {
            //system ("roslaunch agvc_coordinator xsens_driver.launch &" ); //Ideally this will be one launch file
            system ("roslaunch agvc_coordinator AGVCFull.launch &");

            fullActive = true;

            cout << "\033[2;32mAGVC_Coordinator: Full autonomous mode activated\033[0m" << endl;
            writeToLog("Full autonomous mode activated");

            playSound("Autonomy enabled");
        }
        else
        {
            cout << "\033[1;31mAGVC_Coordinator: Another mode is active\033[0m" << endl;
            writeToLog("Another mode is active");

            playSound("Another mode is active");
        }
    }

    /**
      Stops the full autonomous mode
    */
    void stopFull()
    {
        cout << "\033[2;32mAGVC_Coordinator: Stopping full mode\033[0m" << endl;
        writeToLog("Stopping full mode");
        toggleMotorState(true);

        system ("rosnode list > nodes");

        killNodes();

        fullActive = false;

        playSound("Autonomy disabled");
    }

    /**
      Launches the necessary nodes for the partial mode
    */
    void startPartial()
    {
        if (!partialActive && !fullActive)
        {
            system ("roslaunch agvc_coordinator AGVCPartial.launch &");
            //system ("roslaunch hector_pose_estimation hector_pose_estimation.launch &");

            partialActive = true;

            cout << "\033[2;32mAGVC_Coordinator: Partial mode activated\033[0m" << endl;
            writeToLog("Partial mode activated");

            playSound("Manual control enabled");
        }
        else
        {
            cout << "\033[1;31mAGVC_Coordinator: Another mode is active\033[0m" << endl;
            writeToLog("Another mode is active");

            playSound("Another mode is active");
        }
    }

    /**
      Stops the partial mode
    */
    void stopPartial()
    {
        cout << "\033[2;32mAGVC_Coordinator: Stopping partial mode\033[0m" << endl;
        writeToLog("Stopping partial mode");

        toggleMotorState(true);

        system ("rosnode list > nodes");

        killNodes();

        partialActive = false;

        playSound("Manual control disabled");
    }

    /**
      Kills all the nodes found in the "nodes" file
    */
    void killNodes()
    {
        ifstream nodeFile("nodes");
        char allowedNodesFilename[200];
        list<string> allowedNodes;

        if (fullActive)
        {
            sprintf(allowedNodesFilename, "%sagvc_coordinator/config/AutonomousNodesToKeep", path.c_str());
        }
        else
        {
            sprintf(allowedNodesFilename, "%sagvc_coordinator/config/ManualNodesToKeep", path.c_str());
        }

        ifstream allowedNodesFile(allowedNodesFilename);

        if (!nodeFile.is_open())
        {
            cout << "\033[1;31mAGVC_Coordinator: Could not load nodes file\033[0m\n" << endl;
            writeToLog("Could not load nodes file");
        }
        else if (!allowedNodesFile.is_open())
        {
            cout << "\033[1;31mAGVC_Coordinator: Could not load allowed nodes file\033[0m\n" << endl;
            writeToLog("Could not load allowed nodes file");
        }
        else
        {
            string line;

            while(getline(allowedNodesFile, line))
            {
                allowedNodes.push_front(line);
            }

            while(getline(nodeFile, line))
            {
                bool keep = false;

                for(list<string>::const_iterator it = allowedNodes.begin(); it != allowedNodes.end(); it++)
                {
                    if (line == (*it))
                    {
                        keep = true;
                    }
                }

                if (!keep)
                {
                    char buffer[100];
                    sprintf(buffer, "rosnode kill %s", line.c_str());
                    system (buffer);

                    writeToLog(buffer);
                }
            }

            allowedNodesFile.close();
            nodeFile.close();
            system ("rm nodes");
        }
    }

    /**
      Shuts down any remaining nodes (we can't actually kill everything - rosout will remain running)
    */
    void totalShutdown()
    {
        system ("rosnode kill /joy_node");
        system ("rosnode kill /xsens_driver");
    }

    /**
      Starts a bag file to record all active topics
    */
    void startBagfile()
    {
        if (fullActive || partialActive)
        {
            system ("rosbag record -a &");

            bagFileActive = true;

            cout << "\033[2;32mAGVC_Coordinator: Bag file started\033[0m" << endl;
            writeToLog("Bag file started");

            playSound("Bag file started");
        }
        else
        {
            cout << "\033[1;31mAGVC_Coordinator: Nothing to record\033[0m\n" << endl;
            writeToLog("Nothing to record");
        }
    }

    /**
      Stops the currently running bag file
    */
    void stopBagfile()
    {
        if (bagFileActive)
        {
            system ("rosnode list > nodes");

            ifstream nodeFile("nodes");

            if (!nodeFile.is_open())
            {
                cout << "\033[1;31mAGVC_Coordinator: Could not load nodes file\033[0m\n" << endl;
                writeToLog("Could not load nodes file");
            }
            else
            {
                string line;

                while(getline(nodeFile, line))
                {
                    if (line.substr(0, 7) == "/record")
                    {
                        char buffer[100];
                        sprintf(buffer, "rosnode kill %s", line.c_str());
                        system (buffer);
                    }
                }
            }

            nodeFile.close();
            system ("rm nodes &");

            cout << "\033[2;32mAGVC_Coordinator: Bag file stopped\033[0m" << endl;
            writeToLog("Bag file stopped");

            bagFileActive = false;

            playSound("Bag file stopped");
        }
        else
        {
            cout << "\033[1;31mAGVC_Coordinator: No bag file is running\033[0m\n" << endl;
            writeToLog("No bag file is running");
        }
    }

    /**
      Write to the log file
      @param text The text to write
    */
    void writeToLog(string text)
    {
        char logFilename[200];
        ofstream log;

        sprintf(logFilename, "%sagvc_coordinator/logs/Log-%s", path.c_str(), startTime.c_str());

        log.open(logFilename, ios::app);

        if (!log.is_open())
        {
            ROS_ERROR("\033[1;31mAGVC_Coordinator: Could not open log file\033[0m\n");
        }
        else
        {
            log << text << endl;

            log.close();
        }
    }

    /**
      Sets the time that this node was started
    */
    void setStartTime()
    {
        time_t     now = time(0);
        struct tm  tstruct;
        char       buf[80];
        tstruct = *localtime(&now);
        // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
        // for more information about date/time format
        strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);

        startTime = buf;
    }

    /**
      Gets the computer to say the specified sound
    */
    void playSound(string sound)
    {
        sound_play::SoundClient sc;

        sc.say(sound);
    }

    /**
      Shuts the robot up
    */
    void shutup()
    {
        sound_play::SoundClient sc;

        sc.stopAll();
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
