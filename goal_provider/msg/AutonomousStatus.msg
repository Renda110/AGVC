Header header
geometry_msgs/Point goal            #The goal we are heading to

uint8 status
  uint8 GOAL_ACTIVE    = 1            #We are actively heading to a goal
  uint8 GOAL_WAITING   = 2            #We are waiting for a goal to be accepted by move_base
  uint8 GOAL_REJECTED  = 3            #Movebase rejected our goal
  uint8 GOAL_ABORTED   = 4            #Movebase aborted our goal
  uint8 GOAL_REACHED   = 5            #We have reached the goal and can start planning for the next one
  uint8 LOST           = 6            #We are lost and don't know what is happening anymore. This is terminal

std_msgs/String text                         #For extra information
