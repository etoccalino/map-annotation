import rospy
from visualization_msgs.msg import Marker


class ImagePosePair:
    def __init__(self, image, pose):
        self.img = image.data
        self.pose = pose
        self.tags = []

    def write_files(self, name_base):
        photo_name = name_base+'_photo.jpg'
        pose_name = name_base+'_pose.txt'

        photo_file = open(photo_name, "wb")
        photo_file.write(self.img)
        photo_file.close()

        pose_file = open(pose_name, "w")
        pose_file.write(str(self.pose)+'\n')
        pose_file.close()

    def add_tag(self, tag):
        self.tags.append(tag)

    def add_tags(self, tags):
        self.tags += tags

    def create_marker(self, num):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "/map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.id = num
        marker.action = marker.ADD
        marker.ns = 'tags'
        #marker.lifetime = 0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1
        marker.pose = self.pose
        marker.text = '\n'.join(self.tags)
        return marker
