import sys
import csv
import cPickle as pickle


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
    out_file = sys.argv[3]
except IndexError:
    print "USAGE: " + sys.argv[0] + "  <pairs_file> <csv_file> <out_file>"

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

# Save the list to a piclked file.
dump_file = open(out_file, 'wp')
# Protocol 2 uses the new, more efficient, binary format.
pickle.dump(tagged_pairs, dump_file, 2)
# Save all_tags to another file
all_tags_file = open('all_tags.data', 'wp')
pickle.dump(all_tags, all_tags_file, 2)
