import sys
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, SequentialWriter, TopicMetadata
from sensor_msgs.msg import NavSatFix
import builtin_interfaces.msg

def main():
    if len(sys.argv) < 4:
        print("Usage: python fix_bag.py <path> <output_path> <frame_id>")
        sys.exit(1)

    input_bag = sys.argv[1]
    output_bag = sys.argv[2]
    frame_id = sys.argv[3]

    # Initialize storage and converter options
    storage_options = StorageOptions(uri=input_bag, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    # Create a reader for the input bag file
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Create a writer for the output bag file
    writer_storage_options = StorageOptions(uri=output_bag, storage_id='sqlite3')
    writer = SequentialWriter()
    writer.open(writer_storage_options, converter_options)

    # Get metadata for all topics and create a mapping from topic name to its type
    topics = reader.get_all_topics_and_types()
    topic_types = {topic_metadata.name: topic_metadata.type for topic_metadata in topics}
    # Precompute the set of topics that are NavSatFix messages
    navsatfix_topics = {name for name, msg_type in topic_types.items() if "NavSatFix" in msg_type}

    # Create topics in the output bag file
    for topic_metadata in topics:
        topic_info = TopicMetadata(name=topic_metadata.name,
                                type=topic_metadata.type,
                                serialization_format='cdr')
        writer.create_topic(topic_info)

    # Constants for the updated message fields
    covariance = [0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1]
    covariance_type = 2  # e.g., COVARIANCE_TYPE_DIAGONAL_KNOWN

    while reader.has_next():
        topic, data, t = reader.read_next()  # t is the timestamp in nanoseconds
        if topic in navsatfix_topics:
            msg = deserialize_message(data, NavSatFix)
            # Directly assign timestamp values without creating a new Time object
            msg.header.stamp.sec = int(t // 1e9)
            msg.header.stamp.nanosec = int(t % 1e9)
            msg.header.frame_id = frame_id
            msg.position_covariance = covariance
            msg.position_covariance_type = covariance_type
            new_data = serialize_message(msg)
        else:
            new_data = data

        writer.write(topic, new_data, t)

    print("Finished processing bag file. Corrected bag saved as:", output_bag)

if __name__ == '__main__':
    main()