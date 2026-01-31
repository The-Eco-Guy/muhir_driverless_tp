import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np

# Waypoints data
waypoints = np.array(((4, 0.5), (6.0997, 0.46011999999999986), (8.958765, 0.6334599999999995), (12.904501, 0.5442049999999998), (16.50963, 0.9106499999999986), (20.142315, 2.0267799999999987), (23.03038, 3.0184499999999996), (25.728749999999998, 4.120899999999999), (27.0522, 3.8519499999999987), (29.24365, 3.0322499999999994), (30.2343, 2.159699999999999), (31.7488, 0.11077499999999851), (32.04075, -1.3419550000000005), (31.85275, -4.607830000000001), (30.93425, -7.440205000000001), (30.10215, -9.54865), (28.42945, -11.919970000000001), (26.178150000000002, -13.647795), (23.266765, -15.095735000000001), (20.758225, -16.539845), (16.424795, -18.711165), (12.720585, -20.563344999999998), (9.160635, -22.593), (6.340465, -24.4171), (3.4438549999999992, -25.2316), (2.0123599999999993, -24.977), (-0.24815000000000076, -23.59835), (-1.1382499999999993, -22.7739), (-3.0364499999999985, -20.334455), (-3.387450000000001, -16.96707), (-3.5478500000000004, -15.764190000000001), (-3.2800499999999992, -12.895075), (-2.8646499999999993, -10.3), (-2.565950000000001, -7.250210000000001), (-2.5943500000000004, -4.270505000000001), (-1.4700000000000006, -1.7200000000000006), (-0.6999999999999993, -0.6400000000000006), (0.5199999999999996, -0.26000000000000156)))

class VehicleState(Node):
    def __init__(self):
        super().__init__('Vehicle_State')
        
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
        self.subscription = self.create_subscription(Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        self.prev_time = None
        self.current_wp_idx = 0
        self.next_wp_idx = 1
        self.waypoints = waypoints
        self.is_finished = False
        self.final_lap_logged = False  # Added flag to control single publishing

        self.lap_start_time = None

        self.prev_error = 0
        self.e_i = 0
        self.e_d = 0
        self.wheelbase = 1.58
        self.curvature = 0
        
        self.target_velocity = 0
        self.max_velocity =20

        self.pid_kp = 2.6
        self.pid_ki = 0
        self.pid_kd = 0.6

        self.stanley_k = 2.5
        self.stanley_ks = 1.4
        
        # Prediction weights
        self.prediction_weights = [0.85, 0.6, 0.44, 0.27, 0.06, 0.01]
        self.prediction_steps = 6 

    def quaternion_to_euler(self, x, y, z, w):
        t3 = +2 * (w * z + x * y)
        t4 = +1 - 2 * (y * y + z * z)
        return np.arctan2(t3, t4)
    
    def get_current_time(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def calculate_dt(self, current_time):
        dt = 0
        if self.prev_time is not None:
            dt = current_time - self.prev_time
        return dt

    def pid_control(self, error, e_i, e_d):
        return (self.pid_kp * error + (self.pid_ki * e_i) + (self.pid_kd * e_d))

    def stanley_control(self, cross_track_error, heading_error, velocity):
        # Base Stanley steering
        stanley_output = heading_error + np.arctan2(self.stanley_k * cross_track_error, velocity + self.stanley_ks)

        feedforward = self.get_curvature_feedforward(self.current_wp_idx, self.next_wp_idx)
        return stanley_output

    def get_curvature_feedforward(self, current_wp_idx, next_wp_idx):
        idx1 = current_wp_idx
        idx2 = next_wp_idx
        idx3 = (next_wp_idx + 1) % len(self.waypoints)

        p1 = self.waypoints[idx1]
        p2 = self.waypoints[idx2]
        p3 = self.waypoints[idx3]

        a = np.linalg.norm(p2 - p3)
        b = np.linalg.norm(p1 - p3)
        c = np.linalg.norm(p1 - p2)

        area = 0.5 * np.abs(p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1]))

        if area < 1e-6:
            self.curvature = 0
            return 0.0
            
        R = (a * b * c) / (4 * area)
        self.curvature = 1/R
        
        vec1 = p2 - p1
        vec2 = p3 - p2
        cross_prod = np.cross(vec1, vec2)
        
        feedforward = np.arctan(self.wheelbase / R)

        if cross_prod < 0:
            feedforward = -feedforward

        return feedforward

    def waypoint_calculator(self, x, y, current_wp_idx, next_wp_idx):

        wp_curr = self.waypoints[current_wp_idx]
        wp_next = self.waypoints[next_wp_idx]
        
        vector_path = wp_next - wp_curr
        vector_car = wp_next - np.array([x, y])
        
        new_curr_idx = current_wp_idx
        new_next_idx = next_wp_idx
        finished = False

        if np.dot(vector_path, vector_car) <= 0:
            if next_wp_idx == len(self.waypoints) - 1:
                finished = True
            else:
                new_curr_idx = (current_wp_idx + 1) % len(self.waypoints)
                new_next_idx = (next_wp_idx + 1) % len(self.waypoints)
        
        return new_curr_idx, new_next_idx, finished

    def handle_lap_timer_and_finish(self, current_time):
        if self.lap_start_time is None:
            self.lap_start_time = current_time

        if self.is_finished:
            # Only execute the finish logic once
            if not self.final_lap_logged:
                total_lap_time = current_time - self.lap_start_time
                self.get_logger().info(f"Lap finished")
                self.get_logger().info(f"Final Lap Time: {total_lap_time:.3f} seconds")
                
                drive_msg = AckermannDriveStamped()
                drive_msg.drive.speed = 0.0
                drive_msg.drive.acceleration = -10.0 
                drive_msg.drive.jerk = 100.0 
                drive_msg.drive.steering_angle = 0.0
                self.publisher_.publish(drive_msg)
                
                self.final_lap_logged = True
            
            return True 
        
        return False

    def extract_state_from_msg(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        v_z = msg.twist.twist.angular.z
        q = msg.pose.pose.orientation
        current_heading = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        return x, y, v_x, v_y, v_z, current_heading

    def calculate_tracking_errors(self, x, y, current_heading, current_wp_idx, next_wp_idx):
        xf = x + (self.wheelbase) * np.cos(current_heading)
        yf = y + (self.wheelbase) * np.sin(current_heading)
        
        wp_curr = self.waypoints[current_wp_idx]
        wp_next = self.waypoints[next_wp_idx]
        
        desired_heading = np.arctan2(wp_next[1] - wp_curr[1], wp_next[0] - wp_curr[0])
        heading_error = desired_heading - current_heading
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        A = wp_curr
        B = wp_next
        P = np.array([xf, yf])

        AB = B - A
        AP = P - A

        cross_track_error = np.cross(AP, AB) / np.linalg.norm(AB)
        return cross_track_error, heading_error

    def compute_steering_cmd(self, cross_track_error, heading_error):
        v_limit = np.sqrt(20 / np.abs(self.curvature)) if self.curvature != 0 else self.max_velocity
        target_velocity_step = np.clip(v_limit, 0, self.max_velocity)
        
        steering_cmd = self.stanley_control(cross_track_error, heading_error, target_velocity_step) 

        max_steer = 0.4
        steering_cmd = np.clip(steering_cmd, -max_steer, max_steer)
        return steering_cmd

    def compute_accel_cmd(self, v_x, dt):
        error = self.target_velocity - v_x
        self.e_i += error * dt
        self.e_i = np.clip(self.e_i, -5, 5)
        
        self.e_d = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        
        accel_cmd = self.pid_control(error, self.e_i, self.e_d)
        return accel_cmd

    def publish_drive_cmd(self, speed, acceleration, steering_angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.acceleration = float(acceleration)
        drive_msg.drive.steering_angle = float(steering_angle)
        self.publisher_.publish(drive_msg)
    
    def compute_state(self, heading, steering, v_x, v_y, v_z, x, y):
        # Kinematic Bicycle prediction
        dt_pred = 0.1
        
        # Calculate yaw rate
        yaw_rate = (v_x / self.wheelbase) * np.tan(steering)
        next_heading = heading + yaw_rate * dt_pred
        
        # Update position
        next_x = x + v_x * np.cos(heading) * dt_pred
        next_y = y + v_x * np.sin(heading) * dt_pred
        
        return (next_heading, next_x, next_y)

    def predictive_stanley_loop(self, x, y, current_heading, v_x, v_y, v_z):
        curr_x, curr_y, curr_h = x, y, current_heading
        curr_wp_idx = self.current_wp_idx
        next_wp_idx = self.next_wp_idx
        
        final_steering_cmd = 0.0
        n_loops = 6
        
        # Calculating total weight
        total_weight = sum(self.prediction_weights[:n_loops])
        
        first_cte = 0.0

        for i in range(n_loops):

            xf = curr_x + (self.wheelbase) * np.cos(curr_h)
            yf = curr_y + (self.wheelbase) * np.sin(curr_h)
            
            new_curr, new_next, finished = self.waypoint_calculator(xf, yf, curr_wp_idx, next_wp_idx)
            
            if i == 0:
                self.current_wp_idx = new_curr
                self.next_wp_idx = new_next
                self.is_finished = finished
                if self.is_finished:
                    return None, None 
            
            self.get_curvature_feedforward(new_curr, new_next)
            cte, he = self.calculate_tracking_errors(curr_x, curr_y, curr_h, new_curr, new_next)
            
            if i == 0:
                first_cte = cte
                v_limit = np.sqrt(9.5 / np.abs(self.curvature)) if self.curvature != 0 else self.max_velocity
                self.target_velocity = np.clip(v_limit, 0, self.max_velocity)

            steering_cmd = self.compute_steering_cmd(cte, he)
            
            # Applying weighted steering contribution
            weight = self.prediction_weights[i] / total_weight
            final_steering_cmd += steering_cmd * weight
            
            # Predicting next state
            curr_h, curr_x, curr_y = self.compute_state(curr_h, steering_cmd, v_x, v_y, v_z, curr_x, curr_y)
            
            curr_wp_idx = new_curr
            next_wp_idx = new_next
            
        return final_steering_cmd, first_cte

    def odom_callback(self, msg):
        current_time = self.get_current_time(msg)
        
        if self.handle_lap_timer_and_finish(current_time):
            return

        x, y, v_x, v_y, v_z, current_heading = self.extract_state_from_msg(msg)

        dt = self.calculate_dt(current_time)
        if self.prev_time is None or dt <= 0:
            self.prev_time = current_time
            return
        self.prev_time = current_time


        final_steering_cmd, first_cte = self.predictive_stanley_loop(x, y, current_heading, v_x, v_y, v_z)
        
        if self.is_finished:
            return

        # Debugging
        self.get_logger().info(
            f"WP: {self.current_wp_idx} | Lap Time: {current_time - self.lap_start_time:.2f}s | "
            f"CTE: {first_cte:.2f} | Speed: {v_x:.2f}"
        )
        
        accel_cmd = self.compute_accel_cmd(v_x, dt)
        self.publish_drive_cmd(self.target_velocity, accel_cmd, final_steering_cmd)

def main(args=None):
    rclpy.init(args=args)
    vehicle_state = VehicleState()
    rclpy.spin(vehicle_state)
    vehicle_state.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
