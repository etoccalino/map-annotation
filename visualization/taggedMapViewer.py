import sys
import cPickle as pickle

import rospy
from visualization_msgs.msg import MarkerArray

from map_annotation import TaggedMap, MapViz, PoseViz

try:
    res_dir = sys.argv[1]
except IndexError:
    print "USAGE: " + sys.argv[0] + "  <results dir>"
    sys.exit(1)

rospy.init_node('tagviz', anonymous=False)
pub = rospy.Publisher('/tagcloud', MarkerArray)
pub_probs = rospy.Publisher('/tagsprobs', MarkerArray)

pairs = pickle.load(open(res_dir+'/tagged_pairs.data'))
all_tags = pickle.load(open(res_dir+'/all_tags.data'))
tag_map = TaggedMap.load_map(res_dir+'/tagged_map.data')


# Create tag cloud
ma = MarkerArray()

for num, pair in enumerate(pairs):
    pv = PoseViz(pair)
    marker = pv.create_marker(num, all_tags)
    ma.markers.append(marker)

mv = MapViz(tag_map)
tags_probs = mv.get_maker_array(pairs, all_tags)

while not rospy.is_shutdown():
    pub.publish(ma)
    rospy.loginfo("Tags cloud published")
    pub_probs.publish(tags_probs)
    rospy.loginfo("Tags probs published")
    rospy.sleep(1)
