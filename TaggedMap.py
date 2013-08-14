from dynamicmap.DynamicMap import DynamicMap
import numpy


class TaggedMap(DynamicMap):

    def __init__(self, grid_size, all_tags):
        num_tags = len(all_tags)
        # First try with identity matrix as probs.
        low = 5e-10
        high = 1.0 - (low * num_tags)
        ones = numpy.ones((num_tags, num_tags))
        eye = numpy.eye(num_tags)
        #A = (ones - eye) * low + eye * high
        A = eye
        B = (ones - eye) * low + eye * high
        DynamicMap.__init__(self, grid_size, all_tags, all_tags, A, B)
        # Disable ray tracing.
        self.set_update_rays(False)

    def update_scan(self):
        raise NotImplementedError("This method should not be used.\
                                   Use update_tags instead.")

    def update_tags(self, tagged_poses):
        for pose_pair in tagged_poses:
            coord = pose_pair.pose_to_coord()
            for tag in pose_pair.tags:
                self.update_point(coord, tag)
