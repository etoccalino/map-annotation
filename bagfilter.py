import sys
import rosbag
import random

import ImagePosePair

if len(sys.argv) != 3:
    print "USAGE: " + sys.argv[0] + " <bagfile> <output dir>"
    sys.exit(1)

bag = rosbag.Bag(sys.argv[1])

# get first pose:
for topic, msg, t in bag.read_messages():
    if topic == '/robot_pose':
        last_pose = msg
        break

# create the list of pairs
pairs = []
for topic, msg, t in bag.read_messages():
    if topic == '/camera/rgb/image_color/compressed':
        pairs.append(ImagePosePair.ImagePosePair(msg, last_pose))

    if topic == '/robot_pose':
        last_pose = msg

# pick rnd images:
sampledpairs = random.sample(pairs, 10)
outdir = sys.argv[2] + '/'

for num, pair in enumerate(sampledpairs):
    name = outdir + str(num)
    pair.write_files(name)
