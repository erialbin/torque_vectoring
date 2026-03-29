# ROS2 Messages

Messages used in this package and the purpose for each message.

## Input

### `/my_input/path`

- **Type**: `my_msgs/msg/Type`

- **Purpose**: Brief description of the purpose of this message

...

## Output

### `/current_time`

- **Type**: `std_msgs/msg/String`

- **Purpose**: Publishes the current ROS2 time as a formatted string

### `/my_output/steering_angle`

- **Type**: `my_msgs/msg/Type`

- **Purpose**: Brief description of the purpose of this message

...

## Example Topic Commands

List topics:
```bash
ros2 topic list
```

Echo a topic:
```bash
ros2 topic echo /current_time
```

Get topic info:
```bash
ros2 topic info /current_time
```
