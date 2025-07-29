#!/usr/bin/env python3
import rosbag2_py
import csv
import sys
import rclpy.serialization
from sail_msgs.msg import SerialMsg

def bag_to_csv(bag_path, csv_path):
    # Set up storage and converter options (assuming sqlite3 storage)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Use the imported message type.
    SerialData = SerialMsg

    data_rows = []
    start_time = None

    # Iterate through all messages in the bag.
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/serial_data':
            # Deserialize the raw data into a SerialMsg instance.
            msg = rclpy.serialization.deserialize_message(data, SerialData)
            # Convert the builtin_interfaces/Time stamp to a float (seconds).
            time_sec = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            if start_time is None:
                start_time = time_sec  # set first message time as 0 sec reference
            relative_time = time_sec - start_time
            data_rows.append([relative_time, msg.id, msg.data])

    # Write the CSV file with header: stamp,id,data
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp', 'id', 'data'])
        writer.writerows(data_rows)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: {} <bag_directory> [output_csv]".format(sys.argv[0]))
        sys.exit(1)
    bag_dir = sys.argv[1]
    csv_file = sys.argv[2] if len(sys.argv) > 2 else 'output.csv'
    bag_to_csv(bag_dir, csv_file)
    print("CSV file written to:", csv_file)
