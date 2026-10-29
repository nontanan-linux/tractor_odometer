import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
from .kinematic_model import TractorTrailerModel
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray

class NMBOdometer(Node):
    def __init__(self):
        super().__init__('nmb_odometer')
        
        # Parameters updated to match tractor_trailer.xacro dimensions
        self.declare_parameter('L0', 1.400) # Wheelbase
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('trailer_L_bars', [1.53]) # Tractor hitch to drawbar axle
        self.declare_parameter('trailer_L_trls', [1.98]) # Drawbar hitch to trailer axle
        
        # Hitch Calculation Parameters
        self.declare_parameter('tractor_tail_length', 0.0) 
        self.declare_parameter('trailer_tail_length', 0.515) 
        
        # Color Parameters
        self.declare_parameter('color_tractor', [1.0, 0.27, 0.0, 0.5])
        self.declare_parameter('color_trailer', [0.0, 0.0, 1.0, 0.5])
        self.declare_parameter('color_wheel', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('color_drawbar', [0.8, 0.9, 1.0, 1.0])
        
        # Dimension Parameters
        self.declare_parameter('tractor_width', 1.5)
        self.declare_parameter('tractor_front_overhang', 0.85)
        self.declare_parameter('tractor_overhang', 0.7) # Rear axle to hitch
        self.declare_parameter('trailer_width', 1.5)
        self.declare_parameter('trailer_body_len', 2.0)
        self.declare_parameter('trailer_overhang', 0.0) # Dolly hitch is on axle
        self.declare_parameter('wheel_diam', 0.6)
        self.declare_parameter('wheel_width', 0.3)
        self.declare_parameter('track_width', 1.0)
        
        self.L0 = self.get_parameter('L0').value
        self.dt = self.get_parameter('dt').value
        dt = self.dt 
        
        l_bars = self.get_parameter('trailer_L_bars').value
        l_trls = self.get_parameter('trailer_L_trls').value
        
        # Visualization Parameters
        self.tractor_width = self.get_parameter('tractor_width').value
        self.tractor_front_overhang = self.get_parameter('tractor_front_overhang').value
        self.tractor_overhang = self.get_parameter('tractor_overhang').value
        
        self.tractor_len = self.tractor_overhang + self.L0 + self.tractor_front_overhang
        
        self.trailer_width = self.get_parameter('trailer_width').value
        self.trailer_body_len = self.get_parameter('trailer_body_len').value
        self.trailer_overhang = self.get_parameter('trailer_overhang').value
        self.wheel_diam = self.get_parameter('wheel_diam').value
        self.wheel_width = self.get_parameter('wheel_width').value
        self.W = self.get_parameter('track_width').value
        
        tractor_tail = self.get_parameter('tractor_tail_length').value
        trailer_tail = self.get_parameter('trailer_tail_length').value
        
        tractor_hitch_offset = self.tractor_overhang + tractor_tail
        trailer_hitch_offset = self.trailer_overhang + trailer_tail
        
        dh_prevs = [tractor_hitch_offset]
        if len(l_bars) > 1:
            dh_prevs.extend([trailer_hitch_offset] * (len(l_bars) - 1))
        
        self.c_tractor = self.get_parameter('color_tractor').value
        self.c_trailer = self.get_parameter('color_trailer').value
        self.c_wheel = self.get_parameter('color_wheel').value
        self.c_drawbar = self.get_parameter('color_drawbar').value
        
        self.trailers = []
        if len(l_bars) == len(l_trls) == len(dh_prevs):
            for i in range(len(l_bars)):
                self.trailers.append({
                    'L_bar': l_bars[i],
                    'L_trl': l_trls[i],
                    'dh_prev': dh_prevs[i]
                })
        else:
            self.get_logger().error("Trailer parameter arrays must have equal length!")
        
        self.model = TractorTrailerModel(self.L0, self.trailers, dt=dt)
        
        self.state = np.zeros(3 + 2 * len(self.trailers))
        self.wheel_phi = np.zeros(2 + len(self.trailers))
        
        self.sub_odom = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_joints_veh = self.create_publisher(JointState, '/vehicle/joint_states', 10)
        self.pub_markers = self.create_publisher(MarkerArray, 'vehicle_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        from rclpy.clock import Clock, ClockType
        system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.timer = self.create_timer(dt, self.update, clock=system_clock)
        self.v0 = 0.0
        self.delta = 0.0
        
        self.sub_initial = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        
        self.get_logger().info(f'NMB Tractor Odometer Started with {len(self.trailers)} trailers')
        
        # Publish TFs once on startup
        self.update()

    def initial_pose_callback(self, msg):
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.state[2] = math.atan2(siny_cosp, cosy_cosp)
        self.update()

    def odom_callback(self, msg):
        self.v0 = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        if abs(self.v0) > 0.1:
            self.delta = math.atan(omega * self.L0 / self.v0)
        else:
            self.delta = 0.0
            
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.state[2] = math.atan2(siny_cosp, cosy_cosp)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def update(self):
        tractor_state_odom = self.state[0:3].copy()
        new_state = self.model.update(self.state, self.v0, self.delta)
        self.state = new_state
        self.state[0:3] = tractor_state_odom
        
        coords = self.model.get_coordinates(self.state)
        
        timestamp = self.get_clock().now().to_msg()
        self.get_logger().info('Odometer timer triggered', throttle_duration_sec=1.0)
        
        def broadcast_tf(x, y, theta, child_frame, parent_frame='map'):
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            q = self.euler_to_quaternion(0, 0, theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        for i in range(len(self.trailers)):
            idx_dolly = 3 + 3*i
            idx_axle = 4 + 3*i
            p_dolly = coords[idx_dolly]
            p_axle = coords[idx_axle]
            theta_drawbar = self.state[3 + 2*i]
            theta_trailer = self.state[3 + 2*i + 1]
            
            # Broadcast TF for visualization
            broadcast_tf(p_dolly[0], p_dolly[1], theta_drawbar, f'trailer_{i+1}_dolly')
            broadcast_tf(p_axle[0], p_axle[1], theta_trailer, f'trailer_{i+1}_base_link')
        
        v_tr_rear = self.v0
        self.wheel_phi[0] += (v_tr_rear / (self.wheel_diam/2.0)) * self.dt
        v_tr_front = self.v0 / math.cos(self.delta) if math.cos(self.delta) != 0 else self.v0
        self.wheel_phi[1] += (v_tr_front / (self.wheel_diam/2.0)) * self.dt
        
        for i in range(len(self.trailers)):
            self.wheel_phi[2+i] += (self.v0 / (self.wheel_diam/2.0)) * self.dt

        marker_array = MarkerArray()
        marker_id = 0
        
        def create_cube_marker(x, y, theta, lx, ly, lz, r, g, b, a=1.0, frame_id='map'):
            nonlocal marker_id
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = timestamp
            m.ns = "shapes"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = lz/2.0
            q = self.euler_to_quaternion(0, 0, theta)
            m.pose.orientation.x = q[0]
            m.pose.orientation.y = q[1]
            m.pose.orientation.z = q[2]
            m.pose.orientation.w = q[3]
            m.scale.x = lx
            m.scale.y = ly
            m.scale.z = lz
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            return m

        def create_line_marker(p1, p2, width, r, g, b, a=1.0, frame_id='map'):
            nonlocal marker_id
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = timestamp
            m.ns = "lines"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = width
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            pt1 = Point(); pt1.x = p1[0]; pt1.y = p1[1]; pt1.z = 0.2
            pt2 = Point(); pt2.x = p2[0]; pt2.y = p2[1]; pt2.z = 0.2
            m.points.append(pt1)
            m.points.append(pt2)
            return m

        def create_sphere_marker(x, y, radius, r, g, b, a=1.0, frame_id='map'):
            nonlocal marker_id
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = timestamp
            m.ns = "points"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = radius
            m.scale.y = radius
            m.scale.z = radius
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            return m

        def create_cylinder_marker(x, y, theta, phi, diameter, width, r, g, b, a=1.0, frame_id='map'):
            nonlocal marker_id
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = timestamp
            m.ns = "shapes"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = diameter/2.0 
            q = self.euler_to_quaternion(math.pi/2, phi, theta)
            m.pose.orientation.x = q[0]
            m.pose.orientation.y = q[1]
            m.pose.orientation.z = q[2]
            m.pose.orientation.w = q[3]
            m.scale.x = diameter
            m.scale.y = diameter
            m.scale.z = width
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            
            m_spoke = Marker()
            m_spoke.header = m.header
            m_spoke.ns = "spokes"
            m_spoke.id = marker_id
            marker_id += 1
            m_spoke.type = Marker.CUBE
            m_spoke.action = Marker.ADD
            m_spoke.pose = m.pose 
            m_spoke.scale.x = diameter * 0.9 
            m_spoke.scale.y = diameter * 0.1 
            m_spoke.scale.z = width * 1.01   
            m_spoke.color.r = 0.0
            m_spoke.color.g = 0.0
            m_spoke.color.b = 0.0
            m_spoke.color.a = 1.0 
            return [m, m_spoke]

        p0 = coords[0]
        p0_f = coords[1]
        theta0 = self.state[2]
        
        tr_center_offset = self.tractor_len / 2.0 - self.tractor_overhang
        p_tractor_c = p0 + tr_center_offset * np.array([np.cos(theta0), np.sin(theta0)])
        
        marker_array.markers.append(create_cube_marker(p_tractor_c[0], p_tractor_c[1], theta0, 
                                                       self.tractor_len, self.tractor_width, 1.0, 
                                                       self.c_tractor[0], self.c_tractor[1], self.c_tractor[2], self.c_tractor[3]))
        
        def add_wheels(center, angle, phi, steered_angle=0.0):
            wl_pos = center + (self.W/2) * np.array([-np.sin(angle), np.cos(angle)])
            wr_pos = center - (self.W/2) * np.array([-np.sin(angle), np.cos(angle)])
            wa = angle + steered_angle
            ms_l = create_cylinder_marker(wl_pos[0], wl_pos[1], wa, phi, self.wheel_diam, self.wheel_width, 
                                          self.c_wheel[0], self.c_wheel[1], self.c_wheel[2], self.c_wheel[3])
            ms_r = create_cylinder_marker(wr_pos[0], wr_pos[1], wa, phi, self.wheel_diam, self.wheel_width, 
                                          self.c_wheel[0], self.c_wheel[1], self.c_wheel[2], self.c_wheel[3])
            marker_array.markers.extend(ms_l)
            marker_array.markers.extend(ms_r)
            
        add_wheels(p0, theta0, self.wheel_phi[0]) 
        add_wheels(p0_f, theta0, self.wheel_phi[1], self.delta) 

        p_tr_rear_face = p0 - self.tractor_overhang * np.array([np.cos(theta0), np.sin(theta0)])
        h1 = coords[2]
        marker_array.markers.append(create_line_marker(p_tr_rear_face, h1, 0.05, 
                                                       self.c_drawbar[0], self.c_drawbar[1], self.c_drawbar[2], self.c_drawbar[3]))

        for k in range(len(self.trailers)):
            idx_h = 2 + 3*k
            idx_dolly = 3 + 3*k
            idx_axle = 4 + 3*k
            h_curr = coords[idx_h]
            p_dolly = coords[idx_dolly]
            p_axle = coords[idx_axle]
            theta_drawbar = self.state[3 + 2*k]
            theta_trailer = self.state[3 + 2*k + 1]
            
            marker_array.markers.append(create_sphere_marker(h_curr[0], h_curr[1], 0.15, 
                                                             self.c_drawbar[0], self.c_drawbar[1], self.c_drawbar[2], self.c_drawbar[3]))
            
            marker_array.markers.append(create_line_marker(h_curr, p_dolly, 0.05, 
                                                           self.c_drawbar[0], self.c_drawbar[1], self.c_drawbar[2], self.c_drawbar[3]))
            
            tl_center_offset = self.trailer_body_len / 2.0 - self.trailer_overhang
            p_trailer_c = p_axle + tl_center_offset * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
            
            marker_array.markers.append(create_cube_marker(p_trailer_c[0], p_trailer_c[1], theta_trailer,
                                                           self.trailer_body_len, self.trailer_width, 1.0, 
                                                           self.c_trailer[0], self.c_trailer[1], self.c_trailer[2], self.c_trailer[3]))
            
            phi_trl = self.wheel_phi[2+k]
            add_wheels(p_dolly, theta_drawbar, phi_trl)
            add_wheels(p_axle, theta_trailer, phi_trl)
            
            p_tl_rear_face = p_axle - self.trailer_overhang * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
            if k < len(self.trailers) - 1:
                h_next = coords[2 + 3*(k+1)]
                marker_array.markers.append(create_line_marker(p_tl_rear_face, h_next, 0.05, 
                                                               self.c_drawbar[0], self.c_drawbar[1], self.c_drawbar[2], self.c_drawbar[3]))
            else:
                p_stub = p_tl_rear_face - 0.1 * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
                marker_array.markers.append(create_line_marker(p_tl_rear_face, p_stub, 0.05, 
                                                               self.c_drawbar[0], self.c_drawbar[1], self.c_drawbar[2], self.c_drawbar[3]))

        self.pub_markers.publish(marker_array)
        
        def broadcast_joint_tf(x, y, z, roll, pitch, yaw, child_frame, parent_frame):
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = float(z)
            q = self.euler_to_quaternion(roll, pitch, yaw)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        # Broadcast Base Link to Map (This fixes the Missing Root TF Error)
        broadcast_joint_tf(self.state[0], self.state[1], 0.0, 0.0, 0.0, float(self.state[2]), 'base_link', 'map')

        # Broadcast Tractor Wheels
        broadcast_joint_tf(1.400375, 0.497155, 0.270015, 0.0, float(self.wheel_phi[1]), 0.0, 'front_left_wheel', 'base_link')
        broadcast_joint_tf(1.400375, -0.496957, 0.270015, 0.0, float(self.wheel_phi[1]), 0.0, 'front_right_wheel', 'base_link')
        broadcast_joint_tf(0.0, 0.496167, 0.331764, 0.0, float(self.wheel_phi[0]), 0.0, 'rear_left_wheel', 'base_link')
        broadcast_joint_tf(0.0, -0.496167, 0.331764, 0.0, float(self.wheel_phi[0]), 0.0, 'rear_right_wheel', 'base_link')

        # Broadcast Trailer Joints if they exist
        for i in range(len(self.trailers)):
            idx_db = 3 + 2*i
            idx_prev = 2 + 2*i
            angle_db = self.state[idx_db] - self.state[idx_prev]
            
            idx_tr = 3 + 2*i + 1
            angle_tr = self.state[idx_tr] - self.state[idx_db]
            
            if i == 0:
                # Tractor to drawbar
                broadcast_joint_tf(-1.53, 0.0, -0.4, 0.0, 0.0, float(angle_db), 'drawbar_link', 'hitch_link')
                # Drawbar wheels
                broadcast_joint_tf(-0.98, 0.496167, 0.331764, 0.0, float(self.wheel_phi[2+i]), 0.0, 'dl_wheel', 'drawbar_link')
                broadcast_joint_tf(-0.98, -0.496167, 0.331764, 0.0, float(self.wheel_phi[2+i]), 0.0, 'dr_wheel', 'drawbar_link')
                
                # Drawbar to trailer
                broadcast_joint_tf(-1.98, 0.0, -0.7, 0.0, 0.0, float(angle_tr), 'trailer_link', 'drawbar_top_hitch_link')
                # Trailer wheels
                broadcast_joint_tf(0.0, 0.496167, 0.331764, 0.0, float(self.wheel_phi[2+i]), 0.0, 'tl_wheel', 'trailer_link')
                broadcast_joint_tf(0.0, -0.496167, 0.331764, 0.0, float(self.wheel_phi[2+i]), 0.0, 'tr_wheel', 'trailer_link')
                
        # Still publish joint_states just in case
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = timestamp
        msg.name = ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']
        msg.position = [float(self.wheel_phi[1]), float(self.wheel_phi[1]), float(self.wheel_phi[0]), float(self.wheel_phi[0])]
        self.pub_joints.publish(msg)
        self.pub_joints_veh.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NMBOdometer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
