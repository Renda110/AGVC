Header header
geometry_msgs/Point goal            #The goal we are heading to

uint8 status
  uint8 ACTIVE    = 1            #We are actively heading to a goal
  uint8 WAITING   = 2            #We are waiting for a goal to be accepted by move_base
  uint8 REJECTED  = 3            #Movebase rejected our goal
  uint8 ABORTED   = 4            #Movebase aborted our goal
  uint8 REACHED   = 5            #We have reached the goal and can start planning for the next one
  uint8 LOST      = 6            #We are lost and don't know what is happening anymore. This is terminal
  uint8 PREEMPTED = 7            #Another goal took priority over this one
  uint8 UPDATING  = 8            #Updating the goal or not
  uint8 ERROR     = 9            #Goal provider has gone gammy
  uint8 INFO      = 10           #INFO message

std_msgs/String text                         #For extra information
