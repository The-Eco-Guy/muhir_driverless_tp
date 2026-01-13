import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
#output published to this

from nav_msgs.msg import Odometry
#to get the state
#find the eufs specific frame
import math
import numpy as np

waypoints = np.array(((4, 0.5), (6.0997, 0.46011999999999986), (8.958765, 0.6334599999999995), (12.904501, 0.5442049999999998), (16.50963, 0.9106499999999986), (20.142315, 2.0267799999999987), (23.03038, 3.0184499999999996), (25.728749999999998, 4.120899999999999), (27.0522, 3.8519499999999987), (29.24365, 3.0322499999999994), (30.2343, 2.159699999999999), (31.7488, 0.11077499999999851), (32.04075, -1.3419550000000005), (31.85275, -4.607830000000001), (30.93425, -7.440205000000001), (30.10215, -9.54865), (28.42945, -11.919970000000001), (26.178150000000002, -13.647795), (23.266765, -15.095735000000001), (20.758225, -16.539845), (16.424795, -18.711165), (12.720585, -20.563344999999998), (9.160635, -22.593), (6.340465, -24.4171), (3.4438549999999992, -25.2316), (2.0123599999999993, -24.977), (-0.24815000000000076, -23.59835), (-1.1382499999999993, -22.7739), (-3.0364499999999985, -20.334455), (-3.387450000000001, -16.96707), (-3.5478500000000004, -15.764190000000001), (-3.2800499999999992, -12.895075), (-2.8646499999999993, -10.3), (-2.565950000000001, -7.250210000000001), (-2.5943500000000004, -4.270505000000001), (-1.4700000000000006, -1.7200000000000006), (-0.6999999999999993, -0.6400000000000006), (0.5199999999999996, -0.26000000000000156)))

class VehicleState(Node):
    def __init__(self):
        super().__init__('Vehicle_State')
       
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
        self.subscription = self.create_subscription(Odometry,'/ground_truth/odom',self.odom_callback,10)
        
        self.prev_time=None
        self.current_wp_idx=0
        self.next_wp_idx=1
        self.waypoints=waypoints

        self.prev_error=0
        self.e_i=0
        self.e_d=0
        self.wheelbase=1.58
        
        self.target_velocity=4

        # PID Constants
        self.pid_kp = 0.3
        self.pid_ki = 0.002
        self.pid_kd = 0.15

        # Stanley Constants
        self.stanley_k = 0.7
        self.stanley_ks = 0.3

    def pid_control(self, error, e_i, e_d):
        return (self.pid_kp * error + (self.pid_ki * e_i) + (self.pid_kd * e_d))

    def stanley_control(self, cross_track_error, heading_error, velocity):
        return (heading_error + math.atan2(self.stanley_k * cross_track_error, velocity + self.stanley_ks))

    def odom_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v_x=msg.twist.twist.linear.x
        q=msg.pose.pose.orientation
        current_heading=self.quaternion_to_euler(q.x,q.y,q.z,q.w)

        current_time=msg.header.stamp.sec+msg.header.stamp.nanosec *1e-9
        dt=0

        if self.prev_time is not None:
            dt=current_time-self.prev_time
        

        if self.prev_time is None or dt <= 0:
            self.prev_time = current_time
            return
            
        self.prev_time=current_time

        # Updating Waypoints
        self.waypoint_calculator(x, y)

        # PID Calculation 
        error=self.target_velocity-v_x
        self.e_i+=error*dt
        self.e_i=np.clip(self.e_i,-5,5)
        if(dt>0):
            self.e_d=(error-self.prev_error)/dt
        else:
            self.e_d=0
        self.prev_error=error
        
        # PID control value
        accel_cmd = self.pid_control(error, self.e_i, self.e_d)

        # Stanley Calculation
        xf = x+(self.wheelbase)*math.cos(current_heading)
        yf = y+(self.wheelbase)*math.sin(current_heading)
        desired_heading=math.atan2(self.waypoints[self.next_wp_idx][1]-self.waypoints[self.current_wp_idx][1], self.waypoints[self.next_wp_idx][0]-self.waypoints[self.current_wp_idx][0])
        heading_error=desired_heading-current_heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        # Stanley cross track error
        A = self.waypoints[self.current_wp_idx]
        B = self.waypoints[self.next_wp_idx]
        P = np.array([xf, yf])

        # Vectors
        AB = B - A
        AP = P - A

        # Cross track error calculation
        cross_track_error = np.cross(AP, AB) / np.linalg.norm(AB)
        
        # Stanley control value
        steering_cmd = self.stanley_control(cross_track_error, heading_error, v_x)

        # debugging
        # Clamping steering to 50 degrees 
        max_steer = 0.50
        steering_cmd = np.clip(steering_cmd, -max_steer, max_steer)

        self.get_logger().info(
            f"CTE: {cross_track_error:.3f} | "
            f"HeadErr: {heading_error:.3f} | "
            f"Steer: {steering_cmd:.3f} | "
            f"PID_error: {error:.4f}"
        )
       
   
        drive_msg = AckermannDriveStamped()
        
	
        drive_msg.drive.speed = float(self.target_velocity*math.cos(heading_error)) 
        drive_msg.drive.acceleration = float(accel_cmd)
        
        drive_msg.drive.steering_angle = float(steering_cmd)
        self.publisher_.publish(drive_msg)

        
    def quaternion_to_euler(self,x,y,z,w):
        t3 = +2*(w*z+x*y) 
        t4 = +1-2*(y*y+z*z) 
        return math.atan2(t3,t4)
    
    def waypoint_calculator(self,x,y):
        vector_path=np.array([self.waypoints[self.next_wp_idx][0]-self.waypoints[self.current_wp_idx][0], self.waypoints[self.next_wp_idx][1]-self.waypoints[self.current_wp_idx][1]])
        vector_car=np.array([self.waypoints[self.next_wp_idx][0]-x, self.waypoints[self.next_wp_idx][1]-y])
        if(np.dot(vector_path,vector_car)>0):
            return (self.waypoints[self.current_wp_idx])
        else:
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            self.next_wp_idx = (self.next_wp_idx + 1) % len(self.waypoints)
            
           
            self.get_logger().info(f"Waypoint set to {self.current_wp_idx}")
   
            
            return (self.waypoints[self.current_wp_idx])

def main(args=None):
    rclpy.init(args=args)
    vehicle_state = VehicleState()
    rclpy.spin(vehicle_state)
    vehicle_state.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

