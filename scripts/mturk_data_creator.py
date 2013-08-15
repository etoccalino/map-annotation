import subprocess
import sys
import os
import csv

baseurl = "http://201.216.244.17:7000/robot_images/"


def gen_url(_id):
    return baseurl + str(_id) + "_photo.jpg"


def gen_useful(photo_dir):
    files = os.listdir(photo_dir)
    useful = [int(a.split('_')[0]) for a in files]
    args = ['scp']
    args.extend([photo_dir+'/'+a for a in files])
    args.append('tulku@etoccalino:/var/www/robot_images/')
    return useful, args

try:
    photo_dir = sys.argv[1]
    out_file = sys.argv[2]
except IndexError:
    print "USAGE: " + sys.argv[0] + " <photo dir> <out file>"
    sys.exit(1)

# Get useful ids.
useful, args = gen_useful(photo_dir)

# Prepare output
csvfile = open(out_file, 'wb')
datawriter = csv.writer(csvfile, delimiter=',', quotechar='"',
                        quoting=csv.QUOTE_ALL)

# Write the header
datawriter.writerow(['image_url', 'pair_id'])

for num, _id in enumerate(useful):
    datawriter.writerow([gen_url(_id), str(_id)])

# SCP files to the web server
subprocess.call(args)
