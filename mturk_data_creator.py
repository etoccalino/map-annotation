import sys
import csv

usefull = [1581, 158, 1645, 191, 254, 2563, 2746, 2774, 2793, 3431, 3645, 3648,
           3778, 4284, 4289, 4311, 4354, 473, 5049, 5066, 5213, 5306, 5987,
           6001, 6561, 6599, 6640, 6985, 7057, 728, 759, 8692, 970]

baseurl = "http://201.216.244.17:7000/robot_images/"


def gen_url(_id):
    return baseurl + str(_id) + "_photo.jpg"

try:
    out_file = sys.argv[1]
except IndexError:
    print "USAGE: " + sys.argv[0] + "  <outfile>"

# Prepare output
csvfile = open(out_file, 'wb')
datawriter = csv.writer(csvfile, delimiter=',', quotechar='"',
                        quoting=csv.QUOTE_ALL)

# Write the header
datawriter.writerow(['image_url', 'pair_id'])

for num, _id in enumerate(usefull):
    datawriter.writerow([gen_url(_id), str(_id)])
