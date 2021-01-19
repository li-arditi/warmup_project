# warmup_project

### Drive in a Square

**Goal:** make the robot move in a square

*Approach:* I need to make the robot move straight to some amount of time at a fixed velocity, then rotate 90 degrees; repeat this 4 times

*Structure of code:* used object oriented programming to create DriveInSquare() class

    * __init__(self): creates node 'drive_in_square', creates publisher to 'cmd_vel' topic, and tells what to do on shutdown (execute self.shutdown() [below])

    * run(self): what actually makes the robot move

        i. set rate of loop to 5Hz

        ii. set count to 0; keeps track of how many times the while loop is executed (4 times to make each side of the square)
        
        iii. create a Twist() message to move the robot forward at 0.2 m/s
        
        iv. create a Twist() message to stop moving forward and turn robot 90 degrees (later, will make turn occur in 2 sec so make rotate at angular speed of 45 deg/sec [but make sure to have it in radians])
        
        v. execute while loop if program is not shutdown and if iteration of movement (aka count) < 4
            
            a. first for loop is to make robot move forward by publishing the move forward Twist() message; make this happen 20 times so with rate = 5Hz then robot will comtinue moving forward for 4 seconds
            
            b. second for loop is to make robot rotate 90 degrees; with rate = 5Hz, to make rotation happen for 2 seconds have for loop loop 10 times
            
            c. increase count by 1 
    
    * shutdown(self): tells what happens when program is shutdown; i.e. publish an "empty" Twist() message to set all Twist parameters to 0 to stop robot

*Gif of robot executing commands:*

![gif of robot driving in square once]
(drive_in_square.gif)
