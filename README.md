# rosbag_recorder
The rosbag_recorder package can create a node that allows to record     rosbag files sequentially, by starting and stopping the recording with ROS Service call.      An example of a use case is having ROS running continuously during a session of experiments;     It is frequent that an experimenter would like to start/stop recording using an external state machine,     creating multiple recordings from one ROS session.
