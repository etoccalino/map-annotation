import sys
import csv
import cPickle as pickle
from map_annotation import TaggedMap


def get_result(row):
    _id = row['Input.pair_id']
    tags = [row['Answer.Tag1'], row['Answer.Tag2'], row['Answer.Tag3']]
    return _id, tags


def find_ids(tags, all_tags):
    ids = [all_tags.index(t) for t in tags]
    return ids


try:
    pairs_file = sys.argv[1]
    csv_file = sys.argv[2]
    out_dir = sys.argv[3]
except IndexError:
    print "USAGE: " + sys.argv[0] + "  <pairs_file> <csv_file> <out_dir>"
    sys.exit(1)

pairs = pickle.load(open(pairs_file))

results = open(csv_file, 'rb')
results_reader = csv.DictReader(results, delimiter=',')

all_tags = set()

for row in results_reader:
    _id, tags = get_result(row)
    for t in tags:
        all_tags.add(t)

all_tags = list(all_tags)

results = open(csv_file, 'rb')
results_reader = csv.DictReader(results, delimiter=',')

tagged_pairs = []

for row in results_reader:
    _id, tags = get_result(row)
    print _id, tags
    pairs[int(_id)].add_tags(find_ids(tags, all_tags))
    tagged_pairs.append(pairs[int(_id)])

# Create tag Map
tag_map = TaggedMap(0.1, all_tags)
tag_map.update_tags(pairs)

# Save the list to a pickled file.
tagged_file = out_dir+'/tagged_pairs.data'
dump_file = open(tagged_file, 'wp')
pickle.dump(tagged_pairs, dump_file, 2)
# Save all_tags to another file
all_tags_file = out_dir+'/all_tags.data'
dump_file = open(all_tags_file, 'wp')
pickle.dump(all_tags, dump_file, 2)
# Save the map.
map_file = out_dir+'/tagged_map.data'
tag_map.save_map(tag_map, map_file)
