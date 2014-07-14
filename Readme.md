# DMP_ACTION

Provides a simple service that plays a DMP and publishes it to the `/output_trajectory` topic.

## Using the Server

First, start up the server:

```
rosrun dmp_action dmp_motion_server.py
```

If you are running this with Amanda's demo, you will need to remap `output_trajectory`:

```
rosrun dmp_action dmp_motion_server.py /output_trajectory:=/dmp
```

You can use the LoadFile service to load a YAML file describing a learned DMP:

```python
lf = rospy.ServiceProxy('dmp_action_server/load_file', LoadFile)
lf(root + 'sample_data/data.yaml')
```

Then, simply send an action request to the server with a start and end pose.

### Server Use Example

Python example below:


```python
client = actionlib.SimpleActionClient('dmp_action_server',
        dmp_action.msg.RequestMotionAction)

client.wait_for_server()

goal = dmp_action.msg.RequestMotionGoal()
goal.start.position.x = 0.4
goal.start.position.y =  0.112
goal.start.position.z = 1.23
goal.start.orientation.x = 0.753
goal.start.orientation.y = 0.658
goal.start.orientation.z = -0.018
goal.start.orientation.w = -0.021

goal.end.position.x = 0.2
goal.end.position.y =  0.162
goal.end.position.z = 8.23
goal.end.orientation.x = 0.753
goal.end.orientation.y = 0.658
goal.end.orientation.z = -0.018
goal.end.orientation.w = -0.021

client.send_goal(goal)

client.wait_for_result()

print client.get_result()
```

## Services

* **/load_file** loads a DMP file, learned as per the DMP ROS package and saved to a YAML file.

## Notes

This package will almost certainly be replaced by something better; it was hacked together on the first day of the "Code Sprint" and has been largely untouched since then.

## Troubleshooting

It's not that long a program; if you're running into trouble make sure all of the ROS topics are set up correctly and the YAML file is in the right place.

### Contact

This package was written by Chris Paxton (cpaxton3@jhu.edu)
