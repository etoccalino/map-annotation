class ImagePosePair(object):
    def __init__(self, image, pose):
        self.img = image.data
        self.pose = pose
        self.tags = []

    def write_files(self, name_base, write_pose=False):
        photo_name = name_base+'_photo.jpg'
        photo_file = open(photo_name, "wb")
        photo_file.write(self.img)
        photo_file.close()

        if write_pose:
            pose_name = name_base+'_pose.txt'
            pose_file = open(pose_name, "w")
            pose_file.write(str(self.pose)+'\n')
            pose_file.close()

    def add_tag(self, tag):
        """
        Tag should be the index of the tag in the (unknown to this class)
        global list of all tags.
        """
        self.tags.append(tag)

    def add_tags(self, tags):
        """
        Tags should be the indices of the tag in the (unknown to this class)
        global list of all tags.
        """
        self.tags += tags

    def pose_to_coord(self):
        x = self.pose.position.x
        y = self.pose.position.y
        t = self.pose.orientation.z
        return (x, y, t)

    # This methods are needed to use pickle to serialize the
    # ImagePosePair object. ROS msg break pickle.

    def __getstate__(self):
        xp = self.pose.position.x
        yp = self.pose.position.y
        zp = self.pose.position.z

        xo = self.pose.orientation.x
        yo = self.pose.orientation.y
        zo = self.pose.orientation.z
        wo = self.pose.orientation.y

        pose = {'position': (xp, yp, zp),
                'orientation': (xo, yo, zo, wo)}
        info = {'img': self.img, 'pose': pose, 'tags': self.tags}
        return info

    def __setstate__(self, state):
        from geometry_msgs.msg import Pose, Point, Quaternion
        point = Point(*state['pose']['position'])
        quat = Quaternion(*state['pose']['orientation'])
        self.pose = Pose(point, quat)
        self.img = state['img']
        self.tags = state['tags']
