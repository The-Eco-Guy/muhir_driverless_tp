import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np
import casadi as ca
import do_mpc

# Waypoints data for the track
waypoints_data = np.array(((4, 0.5), (6.0997, 0.46011999999999986), (8.958765, 0.6334599999999995), (12.904501, 0.5442049999999998), (16.50963, 0.9106499999999986), (20.142315, 2.0267799999999987), (23.03038, 3.0184499999999996), (25.728749999999998, 4.120899999999999), (27.0522, 3.8519499999999987), (29.24365, 3.0322499999999994), (30.2343, 2.159699999999999), (31.7488, 0.11077499999999851), (32.04075, -1.3419550000000005), (31.85275, -4.607830000000001), (30.93425, -7.440205000000001), (30.10215, -9.54865), (28.42945, -11.919970000000001), (26.178150000000002, -13.647795), (23.266765, -15.095735000000001), (20.758225, -16.539845), (16.424795, -18.711165), (12.720585, -20.563344999999998), (9.160635, -22.593), (6.340465, -24.4171), (3.4438549999999992, -25.2316), (2.0123599999999993, -24.977), (-0.24815000000000076, -23.59835), (-1.1382499999999993, -22.7739), (-3.0364499999999985, -20.334455), (-3.387450000000001, -16.96707), (-3.5478500000000004, -15.764190000000001), (-3.2800499999999992, -12.895075), (-2.8646499999999993, -10.3), (-2.565950000000001, -7.250210000000001), (-2.5943500000000004, -4.270505000000001), (-1.4700000000000006, -1.7200000000000006), (-0.6999999999999993, -0.6400000000000006), (0.5199999999999996, -0.26000000000000156)))

class MPCController:
    def __init__(self, wheelbase):
        self.wheelbase = wheelbase
        self.waypoints = waypoints_data
        self.N = 3
        self.dt = 0.5
        self.max_v = 15
        
        # Cost weights
        self.Q_cte = 60
        self.Q_epsi = 40
        self.Q_v = 1
        self.Q_delta = 0
        self.Q_accel = 0
        self.Q_diff_delta = 2000
        self.Q_final_pos = 0.45
        self.cte_limit = 0
        self.Q_boundary = 0.0

        self.current_wp_idx = 0
        self.next_wp_idx = 1
        self.target_velocity = 0.0

        # Model Definition
        self.model = do_mpc.model.Model('discrete')
        
        _x = self.model.set_variable(var_type='_x', var_name='x')
        _y = self.model.set_variable(var_type='_x', var_name='y')
        _v = self.model.set_variable(var_type='_x', var_name='v')
        _psi = self.model.set_variable(var_type='_x', var_name='psi')

        _accel = self.model.set_variable(var_type='_u', var_name='accel')
        _steer = self.model.set_variable(var_type='_u', var_name='steer')

        # TVPs carry future track info into the symbolic horizon
        self.model.set_variable(var_type='_tvp', var_name='wp_curr_x')
        self.model.set_variable(var_type='_tvp', var_name='wp_curr_y')
        self.model.set_variable(var_type='_tvp', var_name='wp_next_x')
        self.model.set_variable(var_type='_tvp', var_name='wp_next_y')
        self.model.set_variable(var_type='_tvp', var_name='wp_after_x')
        self.model.set_variable(var_type='_tvp', var_name='wp_after_y')
        
        self.model.set_rhs('x', _x + _v * ca.cos(_psi) * self.dt)
        self.model.set_rhs('y', _y + _v * ca.sin(_psi) * self.dt)
        self.model.set_rhs('v', _v + _accel * self.dt)
        self.model.set_rhs('psi', _psi + (_v / self.wheelbase) * ca.tan(_steer) * self.dt)

        self.model.setup()

        # Controller Definition
        self.mpc = do_mpc.controller.MPC(self.model)
        
        setup_mpc = {
            'n_horizon': self.N,
            't_step': self.dt,
            'n_robust': 0,
            'store_full_solution': True,
            'nlpsol_opts':{
                'ipopt.print_level':0,
                'print_time':0,
                'ipopt.sb':'yes'
            }
        }
        self.mpc.set_param(**setup_mpc)

        # Cost function variables
        m_x, m_y, m_psi, m_v = self.model.x['x'], self.model.x['y'], self.model.x['psi'], self.model.x['v']
        m_accel, m_steer = self.model.u['accel'], self.model.u['steer']
        
        m_wcx, m_wcy = self.model.tvp['wp_curr_x'], self.model.tvp['wp_curr_y']
        m_wnx, m_wny = self.model.tvp['wp_next_x'], self.model.tvp['wp_next_y']
        m_wax, m_way = self.model.tvp['wp_after_x'], self.model.tvp['wp_after_y']

        # Front axle projection for CTE calculation
        xf = m_x + self.wheelbase * ca.cos(m_psi)
        yf = m_y + self.wheelbase * ca.sin(m_psi)
        
        # Heading error
        desired_heading = ca.atan2(m_wny - m_wcy, m_wnx - m_wcx)
        epsi = ca.atan2(ca.sin(desired_heading - m_psi), ca.cos(desired_heading - m_psi))
        
        # Cross track error
        A = ca.vertcat(m_wcx, m_wcy)
        B = ca.vertcat(m_wnx, m_wny)
        P = ca.vertcat(xf, yf)
        AB = B - A
        AP = P - A
        cte = (AB[0]*AP[1] - AB[1]*AP[0]) / (ca.norm_2(AB) + 1e-6)
        
        # Curvature-based velocity
        p1, p2, p3 = A, B, ca.vertcat(m_wax, m_way)
        dist_a, dist_b, dist_c = ca.norm_2(p2-p3), ca.norm_2(p1-p3), ca.norm_2(p1-p2)
        area = 0.5 * ca.fabs(p1[0]*(p2[1]-p3[1]) + p2[0]*(p3[1]-p1[1]) + p3[0]*(p1[1]-p2[1]))
        kappa = (4 * area) / (dist_a * dist_b * dist_c + 1e-6)
        v_ref = ca.fmin(ca.sqrt(15.0 / (ca.fabs(kappa/2.0) + 1e-6)), self.max_v)

        # Stage Cost (lterm)
        lterm = self.Q_cte * (cte**2)
        lterm += self.Q_epsi * (epsi**2)
        lterm += self.Q_v * (m_v - v_ref)**2
        lterm += self.Q_accel * (m_accel**2)
        lterm += self.Q_delta * (m_steer**2)
        

        lterm += ca.if_else(ca.fabs(cte) > self.cte_limit, 
                            self.Q_boundary * (ca.fabs(cte) - self.cte_limit), 0.0)


        mterm = self.Q_final_pos * ((m_x - m_wnx)**2 + (m_y - m_wny)**2)
        
        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        self.mpc.set_rterm(accel=0.1, steer=self.Q_diff_delta)

        # Constraints
        self.mpc.bounds['lower', '_u', 'accel'] = -5
        self.mpc.bounds['upper', '_u', 'accel'] = 5
        self.mpc.bounds['lower', '_u', 'steer'] = -0.5
        self.mpc.bounds['upper', '_u', 'steer'] = 0.5

        # TVP Function Link
        self.tvp_template = self.mpc.get_tvp_template()
        self.mpc.set_tvp_fun(self.tvp_fun)
        
        self.mpc.setup()
        self.mpc.x0 = np.zeros((4, 1))
        self.mpc.set_initial_guess()

    def tvp_fun(self, t_now):

        # Use .item() to ensure we have scalars, avoiding shape mismatch in vector math
        x0 = self.mpc.x0
        curr_x = float(x0['x', 0])
        curr_y = float(x0['y', 0])
        curr_v = max(float(x0['v', 0]), 1.0) 
        curr_psi = float(x0['psi', 0])
        
        #Local index tracking for the horizon
        curr_idx = self.current_wp_idx
        next_idx = self.next_wp_idx
        
        sim_x, sim_y, sim_psi = curr_x, curr_y, curr_psi

        for k in range(self.N + 1):
            p1 = self.waypoints[curr_idx % len(self.waypoints)]
            p2 = self.waypoints[next_idx % len(self.waypoints)]
            p3 = self.waypoints[(next_idx + 1) % len(self.waypoints)]
            
            # Populate template
            self.tvp_template['_tvp', k, 'wp_curr_x'] = p1[0]
            self.tvp_template['_tvp', k, 'wp_curr_y'] = p1[1]
            self.tvp_template['_tvp', k, 'wp_next_x'] = p2[0]
            self.tvp_template['_tvp', k, 'wp_next_y'] = p2[1]
            self.tvp_template['_tvp', k, 'wp_after_x'] = p3[0]
            self.tvp_template['_tvp', k, 'wp_after_y'] = p3[1]
            
            #Simulating to check waypoint progress
            sim_x += curr_v * np.cos(sim_psi) * self.dt
            sim_y += curr_v * np.sin(sim_psi) * self.dt
            
            xf = sim_x + self.wheelbase * np.cos(sim_psi)
            yf = sim_y + self.wheelbase * np.sin(sim_psi)
            
            # Check if we passed the next waypoint
            vector_path = p2 - p1
            vector_car = p2 - np.array([xf, yf])
            
            if np.dot(vector_path, vector_car) <= 0:
                curr_idx = (curr_idx + 1) % len(self.waypoints)
                next_idx = (next_idx + 1) % len(self.waypoints)
                
        return self.tvp_template

    def waypoint_calculator(self, x, y, current_wp_idx, next_wp_idx):
        wp_curr, wp_next = self.waypoints[current_wp_idx], self.waypoints[next_wp_idx]
        vector_path = wp_next - wp_curr
        vector_car = wp_next - np.array([x, y])
        lap_just_finished = False
        
        if np.dot(vector_path, vector_car) <= 0:
            if current_wp_idx == len(self.waypoints) - 1: 
                lap_just_finished = True
            current_wp_idx = (current_wp_idx + 1) % len(self.waypoints)
            next_wp_idx = (next_wp_idx + 1) % len(self.waypoints)
        return current_wp_idx, next_wp_idx, lap_just_finished

    def calculate_tracking_errors(self, x, y, current_heading, current_wp_idx, next_wp_idx):
        xf, yf = x + self.wheelbase * np.cos(current_heading), y + self.wheelbase * np.sin(current_heading)
        wp_curr, wp_next = self.waypoints[current_wp_idx], self.waypoints[next_wp_idx]
        
        desired_heading = np.arctan2(wp_next[1] - wp_curr[1], wp_next[0] - wp_curr[0])
        epsi = np.arctan2(np.sin(desired_heading - current_heading), np.cos(desired_heading - current_heading))
        
        # 2D cross product for CTE
        AB = wp_next - wp_curr
        AP = np.array([xf, yf]) - wp_curr
        cte = (AB[0]*AP[1] - AB[1]*AP[0]) / (np.linalg.norm(AB) + 1e-6)
        return cte, epsi

    def get_curvature(self, current_wp_idx, next_wp_idx):
        p1 = self.waypoints[current_wp_idx % len(self.waypoints)]
        p2 = self.waypoints[next_wp_idx % len(self.waypoints)]
        p3 = self.waypoints[(next_wp_idx + 1) % len(self.waypoints)]
        
        a, b, c = np.linalg.norm(p2-p3), np.linalg.norm(p1-p3), np.linalg.norm(p1-p2)
        area = 0.5 * np.abs(p1[0]*(p2[1]-p3[1]) + p2[0]*(p3[1]-p1[1]) + p3[0]*(p1[1]-p2[1]))
        
        if area < 1e-6: 
            return 0.0
        R = (a * b * c) / (4 * area + 1e-6)
        return min(1/R, 0.5)

    def solve(self, current_state, wp_idx, next_wp_idx):
        self.current_wp_idx, self.next_wp_idx = wp_idx, next_wp_idx
        kappa = self.get_curvature(wp_idx, next_wp_idx)
        self.target_velocity = min(np.sqrt(15.0 / ((abs(kappa)/2.0 + 1e-6))), self.max_v)
        
        # State: [x, y, v, psi]
        x0 = np.array(current_state).reshape(-1, 1)
        self.mpc.x0 = x0
        u0 = self.mpc.make_step(x0)
        return u0[0,0], u0[1,0]

class VehicleState(Node):
    def __init__(self):
        super().__init__('Vehicle_State')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
        self.subscription = self.create_subscription(Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        self.current_wp_idx, self.next_wp_idx = 0, 1
        self.wheelbase = 1.58
        self.mpc_ctrl = MPCController(self.wheelbase)
        
        self.start_time = self.get_clock().now()
        self.lap_count = 0
        self.best_lap = float('inf')

    def quaternion_to_euler(self, q):
        t3, t4 = +2.0*(q.w*q.z + q.x*q.y), +1.0 - 2.0*(q.y*q.y + q.z*q.z)
        return np.arctan2(t3, t4)

    def odom_callback(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        psi = self.quaternion_to_euler(msg.pose.pose.orientation)
        v_x = msg.twist.twist.linear.x
        
        xf, yf = x + self.wheelbase * np.cos(psi), y + self.wheelbase * np.sin(psi)

        new_curr, new_next, lap_finished = self.mpc_ctrl.waypoint_calculator(xf, yf, self.current_wp_idx, self.next_wp_idx)
        self.current_wp_idx, self.next_wp_idx = new_curr, new_next
        
        cte, epsi = self.mpc_ctrl.calculate_tracking_errors(x, y, psi, self.current_wp_idx, self.next_wp_idx)
        
        # Solver call
        accel, steer = self.mpc_ctrl.solve([x, y, v_x, psi], self.current_wp_idx, self.next_wp_idx)

        self.get_logger().info(f"CTE:{cte:.2f} | HDG:{epsi:.2f} | V:{v_x:.2f} | WP:{self.current_wp_idx}", throttle_duration_sec=0.1)

        if lap_finished:
            now = self.get_clock().now()
            lt = (now - self.start_time).nanoseconds / 1e9
            self.lap_count += 1
            if lt < self.best_lap: 
                self.best_lap = lt
            self.get_logger().info(f"*** LAP COMPLETED *** | LAP:{self.lap_count} | TIME:{lt:.3f}s | BEST:{self.best_lap:.3f}s")
            self.start_time = now

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.acceleration = float(accel)
        drive_msg.drive.steering_angle = float(steer)
        drive_msg.drive.speed = float(self.mpc_ctrl.target_velocity)
        self.publisher_.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
