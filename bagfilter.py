import sys
import rosbag
import random
import cPickle as pickle

from ImagePosePair import ImagePosePair

try:
    bag = rosbag.Bag(sys.argv[1])
    outdir = sys.argv[2] + '/'
    filename = sys.argv[3]
except IndexError:
    print "USAGE: " + sys.argv[0] + " <bagfile> <output dir> <pickled_file> [number_of_poses]"
    sys.exit(1)

try:
    num_pose = int(sys.argv[4])
except IndexError:
    num_pose = 10

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

# Save the list to a piclked file.
dump_file = open(filename, 'wp')
# Protocol 2 uses the new, more efficient, binary format.
pickle.dump(pairs, dump_file, 2)

# pick rnd images:
sampledpairs = random.sample(pairs, num_pose)

for num, pair in enumerate(sampledpairs):
    name = outdir + str(num).zfill(3)
    pair.write_files(name)
