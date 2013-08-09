import sys
import random

import rosbag
import rospy
from visualization_msgs.msg import MarkerArray

from ImagePosePair import ImagePosePair
from words import words

bag = rosbag.Bag(sys.argv[1])

rospy.init_node('tagviz', anonymous=False)
pub = rospy.Publisher('/tagcloud', MarkerArray)

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

sampledpairs = random.sample(pairs, 10)

ma = MarkerArray()

for num, pair in enumerate(sampledpairs):
    ws = random.sample(words, 3)
    pair.add_tags(ws)
    marker = pair.create_marker(num)
    ma.markers.append(marker)

while not rospy.is_shutdown():
    pub.publish(ma)
    rospy.loginfo("Tags published")
    rospy.sleep(1)
