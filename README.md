# warmup_project

## TASK 1: Drive in a Square

### Description
The goal of this task is to make the robot move in a square path. My approach for this problem was to use the `rospy.rate()` function to keep my loop at a relatively constant rate, which would allow me to use time as the measurement for how long to continue moving forward. Then after a fixed amount of time going forward at a constant velocity I can have the robot stop and turn 90 degrees then repeat those two steps a total of 4 times.

### The Code
I create a class `DriveInSquare()` that creates a node 'drive_in_square' that publishes `Twist()` messages to the `/cmd_vel` topic. The `__inti__(self)` function also initializes `self.shutdown()` which publishes an "empty" `Twist()` message to set all values to zero and stop the ronot on shutdown. 

The code in the `run(self)` function is what tells the robot how to move. First the rate of publishing is set to 5Hz, so 5 `Twist()` messages are published per second. The count variable keeps track of how many times our "go forward, then turn" loop occurs. So if `count < 4` and the program has not been shutdown, publish `move_cmd` (go forward at speed of 0.2 m/s) 20 times. With rate = 5Hz this means the robot will continue moving forward for 4 seconds [20 cycles * (1 second/5 cycles)]. After the 4 seconds, publish `twist_cmd` (rotate at angular velocity of radians(45)/second so will take 2 seconds to turn 90 degrees) 10 times to make the rotation occur for 2 seconds. Increase the count by one and stop once `count = 5`

### Gif of robot executing commands

![gif of robot driving in square once](drive_in_square.gif)

<br/>

## TASK 2: Wall Follower

### Description

The goal of this task is to make the robot find a wall and follow it indefinitely (until the program is shutdown) at a relatively fixed distance away. It should also be able to detect and handle corners. This task has many different parts, so I split it up into subtasks: find the first wall (closest wall) the robot will start following and have it position itself next to and parallel to the wall; follow the wall until it detects a wall in front of it; once a new wall is detected, start rotating until it is next to (and parallel to) the new wall. 

### The Code

To get information about the robot's environment the `FollowWall()` class object subscribes to `/scan` to get `LaserScan` messages and publishes to `/cmd_vel` to make the robot move. The different subtasks are identified by a class variable `self.state`. 

Initially the state is set to `FIND_WALL_STATE` so when the program first starts the robot finds the closest wall to follow, if any. The `find_wall()` function that is called if `self.state` is `FIND_WALL_STATE` uses PID control for both rotating towards the wall and moving forward to get closer to the wall. Once the robot is a goal distance away it rotates to the right until it is more or less parallel to the wall (defined by the shortest distance to the wall being at 90 degrees). At that point the robot is ready to follow the wall so the state is set to `FOLLOW_WALL_STATE`.

When state is `FOLLOW_WALL_STATE`, `follow_wall()` is called and the robot starts moving forward. As it's moving, it constantly checks for two things: if it is at a goal distance away from the wall it's following and if there is a wall in front of it (determined by the value of `data.ranges[0]`). While the robot is following the wall and a new wall is not detected, PID control is used to keep the robot a relatively constant goal distance away. If a wall is detected, the robot will slow down and start to rotate to the right, and the state is set to `CORNER_STATE`. In that case, `handle_corner()` is called and the robot continues rotating until it is parallel to the new wall it had detected (defined as when the shortest distance to the wall is at approximately 90 degrees). Taking into account the distance from the detected wall the robot starts turning and its linear velocity, the implementation relies on the fact that by the time the robot is almost parallel to the new wall, the new wall is the closest wall. Once the robot is parallel to the new wall the state is set back to `FOLLOW_WALL_STATE` and the process repeats.

(`run()` keeps the node running until it is shutdown)

### Gif of robot executing commands

![gif of robot following wall](wall_follower_final.gif)

<br/>

## TASK 3: Person Follower

### Description

The goal of this task is to follow an object from a safe distance away. Similar to the Wall Follower task, the `PersonFollower()` class object subscribes to `/scan` to get `LaserScan` messages and publishes to `/cmd_vel`. Using the `/scan` data my implementation uses PID control to dictate linear and angular speeds. 

### The Code

This has execution code very similar to Task 2: Wall Follower in terms of PID control. I have a `find_closest_obj()` function to find the object the robot should be following (assuming it's in the scanner range). The `process_scan()` function uses that information to calculate distance and angular error to set linear and angular velocities, respectively. I also made it such that if the robot has to rotate a lot to face the object, the linear velocity is slower and angular velocity is faster than if the robot doesn't need to rotate as much. (This is not an essential part of the code but is used to make the path to the object more direct rather than loopy/arched)

### Gif of robot executing commands

![gif of robot following cylinder](person_follower.gif)

<br/>

## CHALLENGES

At first my biggest challenge was figuring out how subscribing and publishing worked in practice. I understood it coneptually, but was having trouble writing code to use it. Also, since I've never used (or even heard of) ROS before this class it took some time to understand how it's used. For the Driving in a Square task I mainly looked at ROS documentation online and at the sample code that was provided for stopping at a wall. The Wall Follower task was the one that took me the longest. At first I was having trouble understanding subscribing and publishing again since we now needed to use data from the `/scan` topic to determine robot movement. I was also having some trouble figuring out how to structure my code. Going to office hours with Pouya was so helpful. He patiently helped me think through the logic for the wall follower and let me screenshare my code. It was so helpful and I was finally able to get my robot to move! Once I got a semi-working version Pouya helped me troubleshoot and we also talked for a while about a way to use the scan data to know the location of both walls at a corner. Even though I ended up going a different route it was still really helpful and made me think of other/different ways to implement the wall follower. Since I dd so much trial and error and thinking through concepts for the wall follower, the person follower was a breeze.


<br/>

## FUTURE WORK

If I had more time (and during my free time I will probably do this), I'd figure out another way to implement the wall follower. When Pouya and I were talking about one way to handle corners he mentioned that you can calculate an angle theta which is an angle that gives information about where the current wall ends. Then from theta you have an idea about where the two walls are. 


<br/>

## TAKEAWAYS

* Break the actions/tasks up into pieces

  * Something I tend to get stuck doing is thinking too far ahead and try to figure out how all the pieces fit together. Instead I need to slow down and first break up the main goal into pieces/steps. Then I can start working on each piece and get it to work correctly before I try to do the next step. Then after I have the pieces then I can figure out how to put them together.

* Pouya is amazing

  * Office hours are really helpful. When I get stuck instead of sitting there for 5+ hours running myself in circles/getting nowhere, I need to stop and go to office hours/post a question

  * The main bullet is Pouya is amazing rather than being about office hours because Pouya will help me even outside of office hours. I can message a question to him on Slack and he's super responsive and patient. Also, he helps in a way that is helpful in the long run; so how to troubleshoot myself and think about/break down the problem, etc. 
