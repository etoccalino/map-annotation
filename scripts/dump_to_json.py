import sys
import json
from map_annotation import TaggedMap


try:
    map_file = sys.argv[1]
    json_file = sys.argv[2]
except IndexError:
    print "USAGE: " + sys.argv[0] + " <map file> <json file>"
    sys.exit(1)

tag_map = TaggedMap.load_map(map_file)
all_tags = tag_map._states

tag_pose = {}

for tag in all_tags:
    x, y, t, _, _, _, = tag_map._get_scatter_points(state=tag, limit=0.3)
    coords = zip(x, y, t)
    tag_pose[tag] = coords[0]

with open(json_file, "w") as output:
    json.dump(tag_pose, output)
