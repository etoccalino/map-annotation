import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import rospy
import random
import colorsys
import numpy


def generate_color(num_colors):
    # No llego hasta el final de la escala porque el ultimo color es como el
    # primero.
    colors = []
    for hue in numpy.linspace(0, 280, num_colors):
        hue = hue / 360
        sat = (90 + random.random() * 10) / 100
        light = (50 + random.random() * 10) / 100
        colors.append(colorsys.hls_to_rgb(hue, light, sat))
    return colors


class MapViz(object):

    def __init__(self, tagmap):
        self.tm = tagmap

    def plot_tag_map(self, tag):
        fig, ax = plt.subplots()
        cm = plt.get_cmap("jet")

        tag_prob = []
        xt = []
        yt = []
        for point, node in self.tm.map_iter():
            prob = node.get_occ_prob()
            tag_prob.append(prob[tag])
            x, y, z = point
            xt.append(x)
            yt.append(y)

        surf = ax.scatter(xt, yt, marker='o', c=tag_prob, cmap=cm)
        fig.colorbar(surf, shrink=0.5, aspect=10)
        plt.show()

    def get_maker_array(self, all_poses, all_tags, z=0.1):
        """
        Marker Array form of Marker of CubeList type.
        Each Markerlist has a different namespace, one per tag.
        Each Marker in the list has a different color, depending in the prob.
        """
        markers = []
        ntags = len(all_tags)
        colors = generate_color(ntags)
        for (_id, tag) in enumerate(all_tags):
            markers.append(TagViz.create_cube_marker(_id, tag))

        for pose in all_poses:
            try:
                point = pose.pose_to_coord()
                probs = self.tm.get_point_occ_prob(point)
            except:
                continue

            for index, prob in enumerate(probs):
                r, g, b = colors[index]
                col = ColorRGBA(r, g, b, 1.0 / prob * float(ntags))
                x, y, t = point
                markers[index].points.append(Point(x, y, z))
                markers[index].colors.append(col)

        print "Generated markers: ", len(markers)
        for m in markers:
            print "With points: ", len(m.points)

        return markers


class PoseViz(object):

    def __init__(self, posepair):
        self.pp = posepair

    def create_marker(self, marker_id, all_tags):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "/map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.id = marker_id
        marker.action = marker.ADD
        marker.ns = 'tags'
        #marker.lifetime = 0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1
        marker.pose = self.pp.pose
        tag_str = [all_tags[i] for i in self.pp.tags]
        marker.text = '\n'.join(tag_str)
        return marker


class TagViz(object):

    @staticmethod
    def create_cube_marker(marker_id, ns):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "/map"
        marker.id = marker_id
        marker.type = marker.CUBE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 0.9
        marker.pose.orientation.w = 1.0
        marker.ns = ns
        return marker

