# rosbag_recorder

The rosbag_recorder package can create a node that allows to record rosbag files sequentially, by starting and stopping the recording with ROS Service call.

An example of a use case is having ROS running continuously during a session of experiments. It is frequent that an experimenter would like to start/stop recording using an external state machine, creating multiple recordings from one ROS session.

## Usage

To start the recording_node from the terminal, use the accompanying launch file. 
The launch file takes as arguments:

- a list of the topics to record;
- the path to a folder in which to store the recordings;
- the prefix for the name of the saved file.

Example:

```console
roslaunch rosbag_recorder recording_manager.launch recorded_topics:="['topic1', 'topic2']" target_folder:=$(rospack find rosbag_recorder)/recordings/dump/ filename_prefix:='name_of_today_session'
```

## Important Notes

The recorder DOES NOT CHECK whether the listed topics are correct. 
This choice has been made as some topics that are meant to be recorded may not have been published yet (e.g. they may be published later a s a result of some part of the experiment).
As such, there is not a straightforward way to know whether a topic has been mispelled or just not published yet.

## To Do

1. Implement the use of a YAML file as an alternative way to list the parameters for the node
