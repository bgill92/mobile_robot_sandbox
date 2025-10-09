#!/usr/bin/env python3

from copy import deepcopy
import sys
import rosbag2_py

# Import ROS message definitions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

# Import deserialization utility
from rclpy.serialization import deserialize_message

def read_bag_data(bag_path):
    # Setup the storage options and converter options
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,            # Path to the folder containing the .db3 file
        storage_id='sqlite3'     # The default storage plugin for ROS 2
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    # Create a SequentialReader
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Retrieve all topics and types
    topics_and_types = reader.get_all_topics_and_types()
    print("Topics and types in this bag:")
    for t in topics_and_types:
        print(f"  {t.name} : {t.type}")

    # Create a dictionary to quickly look up message types by topic
    topic_type_map = {t.name: t.type for t in topics_and_types}

    # Create a dictionary mapping ROS type strings to Python classes
    # Update this dictionary if your bag contains additional message types.
    msg_type_class_map = {
        'nav_msgs/msg/Odometry': Odometry,
        'sensor_msgs/msg/LaserScan': LaserScan,
        'tf2_msgs/msg/TFMessage': TFMessage,
        'geometry_msgs/msg/Twist': Twist
    }

    topics_and_data = {t.name: [] for t in topics_and_types}

    print("\nReading messages from bag...\n")
    while reader.has_next():
        topic, raw_data, timestamp = reader.read_next()
        msg_type_str = topic_type_map.get(topic, None)

        # Print a basic header
        # print(f"---\nTopic: {topic}\nTimestamp (ns): {timestamp}")

        # If we have a known message type, deserialize and print some fields
        if msg_type_str in msg_type_class_map:
            msg_class = msg_type_class_map[msg_type_str]
            msg = deserialize_message(raw_data, msg_class)

            topics_and_data[topic].append(deepcopy(msg))

        else:
            print("Skipping unknown message type or no type mapping available.")

    print("\nDone reading bag.")

    return deepcopy(topics_and_data)


def main():
    bag_path = "./rosbag2_2025_01_02-05_29_39/rosbag2_2025_01_02-05_29_39_0.db3"

    topics_and_data = read_bag_data(bag_path)

if __name__ == '__main__':
    main()
