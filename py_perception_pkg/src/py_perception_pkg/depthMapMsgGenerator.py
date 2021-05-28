#####################################################################################
# DepthMapMsgGenerator takes in a 80*208*5 depth_map array and returns a ROS message
#####################################################################################

from py_perception_pkg.msg import DepthMapMsg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

import numpy as np
import rospy

class DepthMapMsgGenerator:
    
    def __init__(self, depth_map):

        self.depth_map = depth_map
        ## Create message
        self.rows = np.shape(self.depth_map)[0]
        self.cols = np.shape(self.depth_map)[1]
        self.channels = np.shape(self.depth_map)[2]
    
    ## Custom Message
    def genCustomDepthMapMsg(self):

        self.depth_map_msg = DepthMapMsg() # Instantiate depth_map_msg from DepthMapMsg definition
        self.depth_map_msg.depth_map_array.layout.dim = \
          [MultiArrayDimension(), MultiArrayDimension(), MultiArrayDimension()]
        
        ## Populate message
        self.depth_map_msg.depth_map_array.layout.data_offset = 0 # No empty padding at start of data

        self.depth_map_msg.depth_map_array.layout.dim[0].label  = "vertical_pixels"
        self.depth_map_msg.depth_map_array.layout.dim[0].size   = self.rows
        self.depth_map_msg.depth_map_array.layout.dim[0].stride = self.rows*self.cols*self.channels

        self.depth_map_msg.depth_map_array.layout.dim[1].label  = "horizontal_pixels"
        self.depth_map_msg.depth_map_array.layout.dim[1].size   = self.cols
        self.depth_map_msg.depth_map_array.layout.dim[1].stride = self.cols*self.channels

        self.depth_map_msg.depth_map_array.layout.dim[2].label  = "channels"
        self.depth_map_msg.depth_map_array.layout.dim[2].size   = self.channels
        self.depth_map_msg.depth_map_array.layout.dim[2].stride = self.channels
        self.depth_map_msg.depth_map_array.data = [0]*self.rows*self.cols*self.channels

        self.dstride0 = self.depth_map_msg.depth_map_array.layout.dim[0].stride
        self.dstride1 = self.depth_map_msg.depth_map_array.layout.dim[1].stride
        self.dstride2 = self.depth_map_msg.depth_map_array.layout.dim[2].stride
        self.offset = self.depth_map_msg.depth_map_array.layout.data_offset

        for i in range(self.rows):
            for j in range(self.cols):
                for k in range(self.channels):
                    self.depth_map_msg.depth_map_array.data[self.offset + self.dstride1*i + \
                      self.dstride2*j + k] = self.depth_map[i, j, k]

        self.depth_map_msg.header.frame_id="depth_map"
        self.depth_map_msg.header.stamp=rospy.Time.now() # Nb: .rospy.get_time() gave error.
        rospy.loginfo("Created custom DepthMapMsg at time: '{0}'".format(rospy.get_time()))
        np.savetxt('/home/inigo/osd_repos/ims/depth_map_test_files/custom_depth_map_msg_data.txt',
                    self.depth_map_msg.depth_map_array.data, fmt="%s")
        return self.depth_map_msg
