import sys
import random

import rosbag
import rospy
from visualization_msgs.msg import MarkerArray

from ImagePosePair import ImagePosePair
from words import words as all_tags

from TaggedMap import TaggedMap
from MapViz import MapViz, PoseViz

bag = rosbag.Bag(sys.argv[1])

rospy.init_node('tagviz', anonymous=False)
pub = rospy.Publisher('/tagcloud', MarkerArray)
pub_probs = rospy.Publisher('/tagsprobs', MarkerArray)

# get first pose:
for topic, msg, t in bag.read_messages():
    if topic == '/robot_pose':
        last_pose = msg
        break

# create the list of pairs
pairs = []
for topic, msg, t in bag.read_messages():
    if topic == '/camera/rgb/image_color/compressed':
        pairs.append(ImagePosePair(msg, last_pose))

    if topic == '/robot_pose':
        last_pose = msg

sampledpairs = random.sample(pairs, 5000)

ma = MarkerArray()

for num, pair in enumerate(sampledpairs):
    ws = random.sample(all_tags, 1)
    ws_i = [all_tags.index(tag) for tag in ws] * 10
    pair.add_tags(ws_i)
    pv = PoseViz(pair)
    marker = pv.create_marker(num, all_tags)
    ma.markers.append(marker)


tag_map = TaggedMap(0.1, all_tags)

tag_map.update_tags(sampledpairs)

mv = MapViz(tag_map)
tags_probs = mv.get_maker_array(sampledpairs, all_tags)

while not rospy.is_shutdown():
    pub.publish(ma)
    rospy.loginfo("Tags cloud published")
    pub_probs.publish(tags_probs)
    rospy.loginfo("Tags probs published")
    rospy.sleep(1)
