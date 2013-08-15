import sys
import cPickle as pickle

import rospy
from visualization_msgs.msg import MarkerArray

from TaggedMap import TaggedMap
from MapViz import MapViz, PoseViz

try:
    tagged_file = sys.argv[1]
    all_tags_file = sys.argv[2]
except IndexError:
    print "USAGE: " + sys.argv[0] + "  <tagged_file> <all_tags_file>"
    sys.exit(1)

rospy.init_node('tagviz', anonymous=False)
pub = rospy.Publisher('/tagcloud', MarkerArray)
pub_probs = rospy.Publisher('/tagsprobs', MarkerArray)

pairs = pickle.load(open(tagged_file))
all_tags = pickle.load(open(all_tags_file))


# Create tag cloud
ma = MarkerArray()

for num, pair in enumerate(pairs):
    pv = PoseViz(pair)
    marker = pv.create_marker(num, all_tags)
    ma.markers.append(marker)

# Create tag Map
tag_map = TaggedMap(0.1, all_tags)

tag_map.update_tags(pairs)

mv = MapViz(tag_map)
tags_probs = mv.get_maker_array(pairs, all_tags)

while not rospy.is_shutdown():
    pub.publish(ma)
    rospy.loginfo("Tags cloud published")
    pub_probs.publish(tags_probs)
    rospy.loginfo("Tags probs published")
    rospy.sleep(1)
