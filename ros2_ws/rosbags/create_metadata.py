import sqlite3
import yaml

# Path to your fixed DB file
db_path = "dervio_recording_fixed.db3"

# Connect to the SQLite database
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Get topics from the topics table
cursor.execute("SELECT id, name, type FROM topics;")
topics = cursor.fetchall()

# Get overall message info: earliest and latest timestamps, and total message count
cursor.execute("SELECT MIN(timestamp), MAX(timestamp), COUNT(*) FROM messages;")
min_timestamp, max_timestamp, total_messages = cursor.fetchone()
duration_ns = max_timestamp - min_timestamp

# Define default offered QoS profiles
default_qos = ("- history: 3\n"
               "  depth: 0\n"
               "  reliability: 1\n"
               "  durability: 2\n"
               "  deadline:\n"
               "    sec: 9223372036\n"
               "    nsec: 854775807\n"
               "  lifespan:\n"
               "    sec: 9223372036\n"
               "    nsec: 854775807\n"
               "  liveliness: 1\n"
               "  liveliness_lease_duration:\n"
               "    sec: 9223372036\n"
               "    nsec: 854775807\n"
               "  avoid_ros_namespace_conventions: false")
# For example, using a slightly different profile for /rosout
rosout_qos = ("- history: 3\n"
              "  depth: 0\n"
              "  reliability: 1\n"
              "  durability: 1\n"
              "  deadline:\n"
              "    sec: 9223372036\n"
              "    nsec: 854775807\n"
              "  lifespan:\n"
              "    sec: 10\n"
              "    nsec: 0\n"
              "  liveliness: 1\n"
              "  liveliness_lease_duration:\n"
              "    sec: 9223372036\n"
              "    nsec: 854775807\n"
              "  avoid_ros_namespace_conventions: false")

# Build the topics_with_message_count list in the desired format
topics_list = []
for topic_id, topic_name, topic_type in topics:
    # Get the message count for the topic
    cursor.execute("SELECT COUNT(*) FROM messages WHERE topic_id = ?;", (topic_id,))
    count = cursor.fetchone()[0]
    
    # Use a specific QoS profile if desired, e.g. for /rosout
    qos_profile = rosout_qos if topic_name == "/rosout" else default_qos
    
    topics_list.append({
        "topic_metadata": {
            "name": topic_name,
            "type": topic_type,
            "serialization_format": "cdr",
            "offered_qos_profiles": qos_profile
        },
        "message_count": count
    })

# Build the complete metadata structure
metadata = {
    "rosbag2_bagfile_information": {
        "version": 5,
        "storage_identifier": "sqlite3",
        "duration": {
            "nanoseconds": duration_ns
        },
        "starting_time": {
            "nanoseconds_since_epoch": min_timestamp
        },
        "message_count": total_messages,
        "topics_with_message_count": topics_list,
        "compression_format": "",
        "compression_mode": "",
        "relative_file_paths": [db_path],
        "files": [{
            "path": db_path,
            "starting_time": {
                "nanoseconds_since_epoch": min_timestamp
            },
            "duration": {
                "nanoseconds": duration_ns
            },
            "message_count": total_messages
        }]
    }
}

# Write the YAML metadata to a file
with open("metadata.yaml", "w") as f:
    yaml.dump(metadata, f, sort_keys=False)

conn.close()
