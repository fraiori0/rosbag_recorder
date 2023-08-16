#!/usr/bin/env python

import os
from std_srvs.srv import Trigger, TriggerResponse
import rospy
import subprocess
from datetime import datetime

class Recorder:
    def __init__(self, target_folder, prefix, recorded_topics) -> None:
        self.recording = False
        self.folder = str(target_folder)
        self.prefix = str(prefix)
        self.recorded_topics = list(recorded_topics)

        # name of rosbag node that will record, used to kill it
        self.node_name = "rosbag_recorder_node_that_can_be_killed"

        self.start_service = rospy.Service(
            'recording_manager/start_recording', Trigger, self.start_recording)
        self.stop_service = rospy.Service(
            'recording_manager/stop_recording', Trigger, self.stop_recording)

    def start_recording(self, trigger_request):
        if self.recording:
            return TriggerResponse(success=False, message='A recording is already in progress')
        
        suffix = datetime.now().strftime("%d%m%Y%H%M%S")
        path_to_bag = os.path.join(
            self.folder,
            self.prefix + '_' + suffix + '.bag'
        )

        # String of topics to be recorded
        topics = ' '.join(topic for topic in self.recorded_topics)

        # Command to be written to terminal, in order to start rosbag recording
        cmd = f"rosbag record --buffsize=0 --output-name={path_to_bag} --publish --lz4 {topics}  __name:={self.node_name}"

        # Start a subprocess and give the command
        # NOTE: a subprocess is used so that the node does not get stuck into this Service call
        # waiting for the recording to finish
        self.p = subprocess.Popen(cmd, shell=True, close_fds=True)
        #  stdin=None, stdout=None, stderr=None, close_fds=True)

        self.recording = True
        return TriggerResponse(success=True, message='Started recording')

    def stop_recording(self, trigger_request):
        if not self.recording:
            return TriggerResponse(success=False, message='No recording was in progress, nothing stopped')

        # update internal variables
        self.recording = False

        # kill the node that was previously started
        os.system(f"rosnode kill /{self.node_name}")
        # and then kill the process
        self.p.terminate()

        return TriggerResponse(success=True, message='Stopped recording')


if __name__ == '__main__':
    try:
        NODE_NAME = 'recorder'
        # Start node and publishers
        rospy.init_node(NODE_NAME)

        recorded_topics = rospy.get_param('~recorded_topics')
        target_folder = rospy.get_param('~target_folder')
        filename_prefix = rospy.get_param('~filename_prefix', default='dump')

        r = Recorder(target_folder, filename_prefix, recorded_topics)

    except rospy.ROSInterruptException:
        pass
    rospy.spin()
