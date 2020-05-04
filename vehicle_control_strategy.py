#!/usr/bin/env python

import rospy
import carla
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int16
from carla_msgs.msg import CarlaEgoVehicleControl
import math
from global_route_planner import GlobalRoutePlanner
from global_route_planner_dao import GlobalRoutePlannerDAO
from controller import VehiclePIDController

class Waypoint_Path(object):
    
    def __init__(self):    
        #Variables    
        self.detection = None
        self.veh_pos = None
        self.goal = None
        self.velocity = None

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.vehicle = self.world.get_actors().filter('vehicle.tesla.model3')[0]
        self.mp = self.world.get_map()

        rospy.init_node('GPS_localization_node')
        
        #Publishers
        self.waypoint_publisher = rospy.Publisher(
                '/carla/tesla/waypoints', Path, queue_size=1, latch=True)    
        
        carla_goal = carla.Transform()
        carla_goal.location.x = -149.02883911
        carla_goal.location.y = -61.167804
        carla_goal.location.z = 16.921186
        carla_goal.rotation.pitch = 0.000000
        carla_goal.rotation.roll = 0.000000
        carla_goal.rotation.yaw = 270
        
        self.goal = carla_goal
        self.reroute()
        rospy.spin()

    def Actor(self):
        
        self.veh_pos = self.mp.get_waypoint(self.vehicle.get_location())


    def Waypoints(self):
        dist_waypoint = []
        self.Actor()

        for j in range(0, 40):
            dist_waypoint.append(self.veh_pos.next(j+1))
            for i in dist_waypoint[j]:
               self.world.debug.draw_point(i.transform.location, size=0.1, \
                        color=carla.Color(r=0, g=255, b=0), life_time=1.0)

    def reroute(self):	
        self.Actor()
        if self.vehicle is None or self.goal is None:
            self.current_route = None
            self.publish_waypoints()
        else:
            self.current_route = self.calculate_route(self.goal)
        self.publish_waypoints()

    def calculate_route(self, goal):      
        dao = GlobalRoutePlannerDAO(self.mp)
        grp = GlobalRoutePlanner(dao)
        grp.setup()
        route = grp.trace_route(self.veh_pos.transform.location,\
                carla.Location(self.goal.location.x,\
                self.goal.location.y, self.goal.location.z))

        return route

    def publish_waypoints(self):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        if self.current_route is not None:
            for wp in self.current_route:
                pose = PoseStamped()
                pose.pose.position.x = wp[0].transform.location.x
                pose.pose.position.y = -wp[0].transform.location.y
                pose.pose.position.z = wp[0].transform.location.z

                quaternion = tf.transformations.quaternion_from_euler(
                    0, 0, -math.radians(wp[0].transform.rotation.yaw))
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                msg.poses.append(pose)

        self.waypoint_publisher.publish(msg)
        rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))
        self.Control()

    def Detection(self, msg):
        self.detection = msg.data

    def Control(self):
        rate = rospy.Rate(10)
        lon_param = {'K_P':0.5,'K_I': 0.5,'K_D': 0}
        lat_param = {'K_P':1.0,'K_I': 0.3,'K_D': 0}
        vehicle_controller = VehiclePIDController(self.vehicle, lon_param, lat_param)

        i = 0
        for k in range(1, len(self.current_route)):
            self.Distance(i)
            self.Waypoints()
            rospy.Subscriber('/machine_learning/output', Int16, self.Detection)   
            while self.distance > 0.5:
                self.Actor()
                control = vehicle_controller.run_step(15, self.current_route[i][0])
                self.velocity = self.vehicle.get_velocity()
                #print(len(self.current_route))
                
                if self.detection == 11:
                    print('Object detected, apply breaks')
                    msg = CarlaEgoVehicleControl()
                    msg.throttle = 0
                    msg.steer = control.steer
                    msg.brake = 1
                    msg.hand_brake = control.hand_brake
                    msg.reverse = control.reverse
                    msg.gear = 1
                    msg.manual_gear_shift = control.manual_gear_shift
                    self.detection = None

                elif len(self.current_route) -5 <= k <= len(self.current_route):
                    msg = CarlaEgoVehicleControl()
                    msg.throttle = 0
                    msg.steer = control.steer
                    msg.brake = 1
                    msg.hand_brake = control.hand_brake
                    msg.reverse = control.reverse
                    msg.gear = 1
                    msg.manual_gear_shift = control.manual_gear_shift
                    print('You arrived to your destination!!')

                else:
                    msg = CarlaEgoVehicleControl()
                    msg.throttle = control.throttle
                    msg.steer = control.steer
                    msg.brake = control.brake
                    msg.hand_brake = control.hand_brake
                    msg.reverse = control.reverse
                    msg.gear = 1
                    msg.manual_gear_shift = control.manual_gear_shift
                
                self.Publisher(msg)
                rate.sleep()
                self.Distance(i)
            i += 1

    def Distance(self, i):
        self.distance = math.sqrt(math.pow(self.veh_pos.transform.location.x - self.current_route[i][0].transform.location.x, 2) + \
                math.pow(self.veh_pos.transform.location.y - self.current_route[i][0].transform.location.y, 2))


    def Publisher(self, msg):
        publisher = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd',\
                CarlaEgoVehicleControl, queue_size=1)
        publisher.publish(msg)
     

if __name__ == '__main__':
    try:
        Waypoint_Path()
    except KeyboardInterrupt:
        print("Press Crtl-C to terminate while statement")
        pass

