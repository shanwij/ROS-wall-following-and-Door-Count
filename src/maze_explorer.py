
#!/usr/bin/env python

import rospy, time 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



def scan_callback(msg):
    global range_front , rSense_range
    global range_right , x , d , count
    global range_left
    global ranges
    global min_front,front, min_right, right, min_left ,left, rightSense , rSense
    
    ranges = msg.ranges
    range_front[:5] = msg.ranges[5:0:-1]  
    range_front[5:] = msg.ranges[-1:-5:-1]

    range_right = msg.ranges[300:345]

    range_left = msg.ranges[60:15:-1]

    rSense_range = msg.ranges[260:280]


    min_range, obj_range = min( (ranges[obj_range],obj_range) for obj_range in xrange(len(ranges)) )
    min_front, front = min( (range_front[front],front) for front in xrange(len(range_front)) )
    min_right, right = min( (range_right[right],right) for right in xrange(len(range_right)) )
    min_left , left  = min( (range_left [left ],left ) for left  in xrange(len(range_left )) )
    rightSense , rSense  = min( (rSense_range [rSense ],rSense ) for rSense  in xrange(len(rSense_range )) )

# decide paths
def finalStep(count):
    global doors
    complete = 0
    doors = count
    t = 10
    print "Door count :", doors
    print "Path = ", doors

    while t:
        if(doors == 1):
            msg.angular.z = 0.35
            msg.linear.x = 0.25

        elif(doors == 2):
            if(t > 5):
                msg.angular.z = 0.3
                msg.linear.x = 0.3
            else:
                msg.angular.z = -0.4
                msg.linear.x = 0.3

        elif(doors == 3):
            msg.angular.z = -0.35
            msg.linear.x = 0.25

        mins, secs = divmod(t, 60)
        timer = '{:02d}:{:02d}'.format(mins, secs)
        time.sleep(1)
        t -= 1

        pub_.publish(msg)

# stop the robot
    else: 
        msg.angular.z = 0
        msg.linear.x = 0
        print("End")
        pub_.publish(msg)



pos_inf = float('inf')
range_front = []
range_right = []
range_left  = []
min_front = 0
front = 0
min_right = 0
right = 0
min_left = 0
left = 0

x = 0  # door count
d = 0  # walls

pub_ = rospy.Publisher('cmd_vel', Twist, queue_size = 1) 
sub_ = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.init_node('maze_explorer')

msg = Twist()
msg.linear.x = 0.0
msg.angular.z = 0.0
        
rate = rospy.Rate(10)
time.sleep(1) 

front_wall = 0 #front wall yes = 0 | no = 1
finalStepStart = 0 #final step start = 1 | not yet = 0

print("Turning robot ......")
msg.angular.z = -0.3
msg.linear.x = 0.3
pub_.publish(msg)
time.sleep(2)
       
while(finalStepStart == 0 and not rospy.is_shutdown()):
    
    while(front_wall == 0): 
        if(min_front > 0.2): 
            if(min_right > 0.18 and not min_right== pos_inf):  
                print("Range: {:.2f}m - Wall-following; turn right.".format(min_right))
                print("Range: {:.2f}m - front".format(min_front))
                msg.angular.z = -0.8
                msg.linear.x = 0.15

            elif(min_front == pos_inf and not min_right == pos_inf and min_left == pos_inf):    
                front_wall = 1

            elif(min_right < 0.1):
                msg.angular.z = 1
                msg.linear.x = -0.1
            else:
                print("Range: {:.2f}m - Wall-following; turn left.".format(min_left))
                print("Range: {:.2f}m - front".format(min_front))
                msg.angular.z = 0.8
                msg.linear.x = 0.15
                
        else:  
            print("Front obstacle. turn")
            msg.angular.z = 1.0
            msg.linear.x = 0.0
            pub_.publish(msg)
            while(min_front < 0.3 and not rospy.is_shutdown()):      
                pass
        
        pub_.publish(msg)

    else:
        print("maze End")
        if(min_front > 0.2): 
            if(min_right > 0.18 and not min_right == pos_inf): 
                print("Range: {:.2f}m - right".format(min_right))
                msg.angular.z = -0.1
                msg.linear.x = 0.2

            else:
                print("Range: {:.2f}m - left".format(min_left))
                msg.angular.z = 0.2
                msg.linear.x = 0.2

        else:  
            print("Front obstacle. turn")
            msg.angular.z = 1.0
            msg.linear.x = 0.0
            pub_.publish(msg)
            while(min_front < 0.3 and not rospy.is_shutdown()):      
                pass
                
        if(min_front == pos_inf and min_right == pos_inf and min_left == pos_inf):
            msg.angular.z = 0
            msg.linear.x = 0 
            finalStep(x)
            finalStepStart = 1
        
        elif(rightSense == pos_inf):
            if(x == 0):
                x = 1
            elif(d == 1):
                x = 2
            elif(d == 2):
                x = 3
            
            print("door detect")

        if(x == 1 and not rightSense == pos_inf):
            d = 1 

        if(x == 2 and not rightSense == pos_inf):
            d = 2

        
        pub_.publish(msg)
    
    rate.sleep()

 

