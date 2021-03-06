 #!/usr/bin/env python

import os
from std_srvs.srv import Trigger, TriggerResponse
import rospy
import subprocess


class Recorder:
    def __init__(self, target_folder, prefix, recorded_topics) -> None:
        self.recording = False
        self.__i = 0
        self.folder = str(target_folder)
        self.prefix = str(prefix)
        self.recorded_topics = list(recorded_topics)

        # name of rosbag node that will record, used to kill it
        self.node_name = "recorder_bag_kill_me_plis"

        self.start_service = rospy.Service('start_recording', Trigger, self.start_recording)
        self.stop_service = rospy.Service('stop_recording', Trigger, self.stop_recording)

    
    def start_recording(self, trigger_request):
        if self.recording:
            return TriggerResponse(success=False, message='Already recording')

        path_to_bag = os.path.join(
            self.folder,
            self.prefix + '_' + str(self.__i).zfill(3) + '.bag'
        )

        # String of topics to be recorded
        topics = ' '.join(topic for topic in self.recorded_topics)

        # Command to write to terminal to start rosbag recording
        cmd = f"rosbag record --buffsize=0 --output-name={path_to_bag} --publish --lz4 {topics}  __name:={self.node_name}"

        # Start a subprocess and give the command
        # NOTE: subprocess is used so that the node does not get stuck into this Service call
        # waiting for the recording to finish
        self.p = subprocess.Popen(cmd, shell=True,close_fds=True)
            #  stdin=None, stdout=None, stderr=None, close_fds=True)

        self.recording=True
        print("eheheheh")
        return TriggerResponse(success=True, message='Started recording')

    def stop_recording(self, trigger_request):
        if not self.recording:
            return TriggerResponse(success=False, message='Already not recording')

        # update internal variables
        self.recording = False
        self.__i += 1

        # kill the node
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