#!/usr/bin/env python

import rospy
import math
import tf
import json
import numpy as np
import pandas as pd
from hsrb_interface import geometry
from hsrb_interface import Robot
#from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Quaternion

robot = Robot()
base = robot.try_get('omni_base')
tts = robot.try_get('default_tts')
whole_body = robot.try_get('whole_body')

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 1)

def marker(x1, y1, x2, y2, n):
    print(f'{n} Marker Published between point ({y1}, {x1}) and ({y2}, {x2})')
    # marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 100)

    marker = Marker()

    marker.header.frame_id = "my_frame"
    marker.header.stamp = rospy.Time.now()
    
    #set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 4
    marker.id = n

    marker.action = 0

    # Set the scale of the marker
    marker.scale.x = 0.03
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Set the pose of the marker
    #marker.pose.position.x = y1
    #marker.pose.position.y = x1
    #marker.pose.position.z = 0.01
    # marker.pose.orientation.x = 0.0
    # marker.pose.orientation.y = 0.0
    # marker.pose.orientation.z = 0.0
    # marker.pose.orientation.w = 1.0

    p = Point()
    p.x = -y1
    p.y = -x1
    p.z = 0.01
    marker.points.append(p)

    q = Point()
    q.x = -y2
    q.y = -x2
    q.z = 0.01
    marker.points.append(q)

    marker_pub.publish(marker)


def move_to_table():
    try:
        #base.go_rel(0, 0, 1.57, 10)
        whole_body.move_to_neutral()
        whole_body.move_to_joint_positions({'arm_lift_joint': 0.30})
        whole_body.end_effector_frame = u'hand_palm_link'
        whole_body.move_end_effector_pose(geometry.pose(), 'my_frame')
        whole_body.move_end_effector_pose(geometry.pose(), 'my_frame')
        #base.go_rel(1, 0, 0, 10)
        #whole_body.move_to_joint_positions({'arm_flex_joint': -1.7})
        #whole_body.move_to_joint_positions({'wrist_roll_joint': -1.5})

    except:
        rospy.logerr('Fail go')

    print('rotating')


def draw_line(x1, y1, x2, y2):
    #x1, y1, x2 & y2 should all be normalized 0 - 0.28 --- 0.36 is the maximum allowed in y axis

    # x1 & y1 are the start coordinates
    # x2 & y2 are the end coordinates

    whole_body.looking_hand_constraint = True
    whole_body.linear_weight = 100.0

    whole_body.move_end_effector_pose(geometry.pose(x = -y1, y = -x1, z=-0.01), 'my_frame')
    whole_body.move_end_effector_pose(geometry.pose(x = -y1, y = -x1, z=0.01), 'my_frame')
    whole_body.move_end_effector_pose(geometry.pose(x = -y2, y = -x2, z=0.01), 'my_frame')
    whole_body.move_end_effector_pose(geometry.pose(x = -y2, y = -x2, z=-0.01), 'my_frame')

    # x_dist = x2 - x1
    # y_dist = y2 - y1

    # sum_dist = x_dist + y_dist
    
    # x = x_dist / sum_dist
    # y = y_dist / sum_dist

    # dist = math.sqrt(pow(x_dist,2)+pow(y_dist,2))

    # whole_body.move_end_effector_by_line((0, y, x), dist)

def normalize(arr, t_min, t_max):
  norm_arr = []
  diff = t_max - t_min
  diff_arr = max(arr) - min(arr)   
  for i in arr:
      temp = (((i - min(arr))*diff)/diff_arr) + t_min
      norm_arr.append(temp)
  return norm_arr

def get_drawing_points(file_path):
  f = open(file_path,)

  data = json.load(f)

  for i in data['people']:
    a = i['pose_keypoints_2d']
    #print(a)
    a1 = np.delete(a, slice(2, None, 3))
    range_to_normalize = (-0.3,0.3)
    normalized_array_1d = normalize(a1, range_to_normalize[0], range_to_normalize[1])
    #print(normalized_array_1d) 
    points = np.reshape(normalized_array_1d, (25, 2))
    
  df = pd.DataFrame(points, columns = ['x', 'y'])
  #df1 = df.drop(columns=['c'])
  #df_norm = (df1-df1.min())/(df1.max()-df1.min())
  
  print(df)

  draw_line(df.loc[17,'x'],df.loc[17,'y'], df.loc[15,'x'], df.loc[15,'y'])
  marker(df.loc[17,'x'],df.loc[17,'y'], df.loc[15,'x'], df.loc[15,'y'], 1)
  draw_line(df.loc[15,'x'],df.loc[15,'y'], df.loc[0,'x'], df.loc[0,'y'])
  marker(df.loc[15,'x'],df.loc[15,'y'], df.loc[0,'x'], df.loc[0,'y'], 2)
  draw_line(df.loc[0,'x'],df.loc[0,'y'], df.loc[16,'x'], df.loc[16,'y'])
  marker(df.loc[0,'x'],df.loc[0,'y'], df.loc[16,'x'], df.loc[16,'y'], 3)
  draw_line(df.loc[16,'x'],df.loc[16,'y'], df.loc[18,'x'], df.loc[18,'y'])
  marker(df.loc[16,'x'],df.loc[16,'y'], df.loc[18,'x'], df.loc[18,'y'], 4)
  draw_line(df.loc[0,'x'],df.loc[0,'y'], df.loc[1,'x'], df.loc[1,'y'])
  marker(df.loc[0,'x'],df.loc[0,'y'], df.loc[1,'x'], df.loc[1,'y'], 5)
  draw_line(df.loc[1,'x'],df.loc[1,'y'], df.loc[2,'x'], df.loc[2,'y'])
  marker(df.loc[1,'x'],df.loc[1,'y'], df.loc[2,'x'], df.loc[2,'y'], 6)
  draw_line(df.loc[2,'x'],df.loc[2,'y'], df.loc[3,'x'], df.loc[3,'y'])
  marker(df.loc[2,'x'],df.loc[2,'y'], df.loc[3,'x'], df.loc[3,'y'], 7)
  draw_line(df.loc[3,'x'],df.loc[3,'y'], df.loc[4,'x'], df.loc[4,'y'])
  marker(df.loc[3,'x'],df.loc[3,'y'], df.loc[4,'x'], df.loc[4,'y'], 8)
  draw_line(df.loc[1,'x'],df.loc[1,'y'], df.loc[5,'x'], df.loc[5,'y'])
  marker(df.loc[1,'x'],df.loc[1,'y'], df.loc[5,'x'], df.loc[5,'y'], 9)
  draw_line(df.loc[5,'x'],df.loc[5,'y'], df.loc[6,'x'], df.loc[6,'y'])
  marker(df.loc[5,'x'],df.loc[5,'y'], df.loc[6,'x'], df.loc[6,'y'], 10)
  draw_line(df.loc[6,'x'],df.loc[6,'y'], df.loc[7,'x'], df.loc[7,'y'])
  marker(df.loc[6,'x'],df.loc[6,'y'], df.loc[7,'x'], df.loc[7,'y'], 11)
  draw_line(df.loc[1,'x'],df.loc[1,'y'], df.loc[8,'x'], df.loc[8,'y'])
  marker(df.loc[1,'x'],df.loc[1,'y'], df.loc[8,'x'], df.loc[8,'y'], 12)
  draw_line(df.loc[8,'x'],df.loc[8,'y'], df.loc[9,'x'], df.loc[9,'y'])
  marker(df.loc[8,'x'],df.loc[8,'y'], df.loc[9,'x'], df.loc[9,'y'], 13)
  draw_line(df.loc[9,'x'],df.loc[9,'y'], df.loc[10,'x'], df.loc[10,'y'])
  marker(df.loc[9,'x'],df.loc[9,'y'], df.loc[10,'x'], df.loc[10,'y'], 14)
  draw_line(df.loc[10,'x'],df.loc[10,'y'], df.loc[11,'x'], df.loc[11,'y'])
  marker(df.loc[10,'x'],df.loc[10,'y'], df.loc[11,'x'], df.loc[11,'y'], 15)
  draw_line(df.loc[11,'x'],df.loc[11,'y'], df.loc[22,'x'], df.loc[22,'y'])
  marker(df.loc[11,'x'],df.loc[11,'y'], df.loc[22,'x'], df.loc[22,'y'], 16)
  draw_line(df.loc[22,'x'],df.loc[22,'y'], df.loc[23,'x'], df.loc[23,'y'])
  marker(df.loc[22,'x'],df.loc[22,'y'], df.loc[23,'x'], df.loc[23,'y'], 17)
  draw_line(df.loc[8,'x'],df.loc[8,'y'], df.loc[12,'x'], df.loc[12,'y'])
  marker(df.loc[8,'x'],df.loc[8,'y'], df.loc[12,'x'], df.loc[12,'y'], 18)
  draw_line(df.loc[12,'x'],df.loc[12,'y'], df.loc[13,'x'], df.loc[13,'y'])
  marker(df.loc[12,'x'],df.loc[12,'y'], df.loc[13,'x'], df.loc[13,'y'], 19)
  draw_line(df.loc[13,'x'],df.loc[13,'y'], df.loc[14,'x'], df.loc[14,'y'])
  marker(df.loc[13,'x'],df.loc[13,'y'], df.loc[14,'x'], df.loc[14,'y'], 20)
  draw_line(df.loc[14,'x'],df.loc[14,'y'], df.loc[19,'x'], df.loc[19,'y'])
  marker(df.loc[14,'x'],df.loc[14,'y'], df.loc[19,'x'], df.loc[19,'y'], 21)
  draw_line(df.loc[19,'x'],df.loc[19,'y'], df.loc[20,'x'], df.loc[20,'y'])
  marker(df.loc[19,'x'],df.loc[19,'y'], df.loc[20,'x'], df.loc[20,'y'], 22)
  whole_body.move_to_neutral()
  #draw_line(df.loc[0,'x'],df.loc[0,'y'], df.loc[1,'x'], df.loc[1,'y'])
  #draw_line(df.loc[0,'x'],df.loc[0,'y'], df.loc[1,'x'], df.loc[1,'y'])
    

if __name__ == '__main__':
    #base.follow_trajectory([geometry.pose(x=1.0, y=0.0, ek=0.0), geometry.pose(x=1.0, y=1.0, ek=math.pi)], time_from_starts=[10, 30], ref_frame_id='base_footprint')
    #collision_world.add_box(x=0.3, y=0.3, z=0.3, pose=geometry.pose(x=1.0, z=0.15), frame_id='map')
    move_to_table()
    #draw_line(0.0,0.0,0.3,0.1)
    get_drawing_points('/workspace/rabr/Project/istockphoto-1065183698-612x612_keypoints (1).json')
    #draw_line(0.3, 0.1, 0.0, 0.0)
    #draw_line(-0.1,0.2,0.1,-0.1)
    #scan()
    #get_drawing_points('/workspace/rabr/Project/istockphoto-1065183698-612x612_keypoints (1).json')
    rospy.spin()
    