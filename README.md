# warmup_project

## TASK 1: Drive in a Square

### Description
The goal of this task is to make the robot move in a square path. My approach for this problem was to use the rospy.rate() function to keep my loop at a relatively constant rate, which would allow me to use time as the measurement for how long to continue moving forward. Then after a fixed amount of time going forward at a constant velocity I can have the robot stop and turn 90 degrees then repeat those two steps a total of 4 times.

### The Code
I create a class DriveInSquare() that creates a node 'drive_in_square' that publishes Twist messages to the 'cmd_vel' topic. The \_\_inti\_\_(self) function also initializes self.shutdown() which publishes an "empty" Twist() message to set all values to zero and stop the ronot on shutdown. 

The code in the run(self) function is what tells the robot how to move. First the rate of publishing is set to 5Hz, so 5 Twist() messages are published per second. The count variable keeps track of how many times our "go forward, then turn" loop occurs. So if count < 4 and the program has not been shutdown, publish the 'move_cmd' (go forward at speed of 0.2 m/s) 20 times. With rate = 5Hz this means the robot will continue moving forward for 4 seconds [20 cycles * (1 second/5 cycles)]. After the 4 seconds, publish the 'twist_cmd' (rotate at angular velocity of radians(45)/second so will take 2 seconds to turn 90 degrees) 10 times to make the rotation occur for 2 seconds. Increase the count by one and stop once count = 5

### Gif of robot executing commands

![gif of robot driving in square once](drive_in_square.gif)

<br/>

## TASK 2: Wall Follower

### Description

### The Code

### Gif of robot executing commands

![gif of robot following wall](wall_follower_final.gif)

<br/>

## TASK 3: Person Follower

### Description

### The Code

### Gif of robot executing commands

![gif of robot following cylinder](person_follower.gif)

<br/>

## CHALLENGES

Describe the challenges you faced programming these robot behaviors and how you overcame them.


<br/>

## FUTURE WORK

If you had more time, how would you improve your robot behaviors? 


<br/>

## TAKEAWAYS

What are your key takeaways from this project that would help you/others in future robot programming assignments? For each takeaway, provide a few sentences of elaboration.