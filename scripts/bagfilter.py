import sys
import rosbag
import random
import cPickle as pickle

from map_annotation import ImagePosePair

try:
    bag = rosbag.Bag(sys.argv[1])
    filename = sys.argv[2]
except IndexError:
    print "USAGE: " + sys.argv[0] + " <bagfile> <pickled_file> [output dir] \
[number_of_poses]"
    print "This script generates a Pickled file with the list of photos and \
poses to be tagged. If a [number_of_poses] is given it also \
generates jpgs and txt files for a random set of poses, in the \
[output_dir]."
    sys.exit(1)

try:
    outdir = sys.argv[3] + '/'
    num_pose = int(sys.argv[4])
except IndexError:
    num_pose = None

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

if num_pose is not None:
    # pick rnd images:
    pairs_indices = range(len(pairs))
    sampledpairs = random.sample(pairs_indices, num_pose)

    for num in sampledpairs:
        name = outdir + str(num).zfill(3)
        pair = pairs[num]
        pair.write_files(name)
