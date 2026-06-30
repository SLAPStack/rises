#!/usr/bin/env python3
"""
Extract warehouse map data from rosbag files.

This script reads a rosbag file and extracts:
1. Warehouse contours (boundaries/walls)
2. Initial obstacle map (pallets with INSERT operations)

Output JSON files can be used by geofence nodes for pre-initialization,
eliminating the need for buffering in the translator.

Usage:
    python3 extract_map_data.py /path/to/bagfile -o /output/dir
    python3 extract_map_data.py /path/to/bagfile --obstacles-only
    python3 extract_map_data.py /path/to/bagfile --contours-only
"""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, asdict
from datetime import datetime


@dataclass
class Obstacle:
    """Represents an obstacle (pallet) with AABB bounds"""
    id: int
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def center_x(self) -> float:
        return (self.x_min + self.x_max) / 2.0

    @property
    def center_y(self) -> float:
        return (self.y_min + self.y_max) / 2.0


@dataclass
class WarehouseContours:
    """Represents warehouse boundary contours"""
    outer_contour_hull: List[List[float]]  # [[x, y], ...]
    outer_contour_segments: List[List[List[float]]]  # [[[x1, y1], [x2, y2]], ...]
    inner_contours: List[List[List[List[float]]]]  # polygon segments


class MapDataExtractor:
    """Extract map data from rosbag files"""

    def __init__(self, bag_path: str):
        self.bag_path = Path(bag_path)
        self.obstacles: Dict[int, Obstacle] = {}  # id -> obstacle
        self.warehouse_contours: Optional[WarehouseContours] = None
        self.message_counts = {
            'obstacles': 0,
            'contours': 0,
            'inserts': 0,
            'deletes': 0,
        }

    def extract(self, topics: Optional[List[str]] = None) -> bool:
        """
        Extract data from rosbag.

        Args:
            topics: List of topics to extract. If None, extracts all map-related topics.

        Returns:
            True if extraction successful, False otherwise.
        """
        if topics is None:
            topics = ['/mqtt/discerning_safety_map', '/mqtt/warehouse_contours']

        # Try direct sqlite3 extraction first (works with any metadata version)
        if self._extract_sqlite_direct(topics):
            return True

        # Fall back to rosbag2_py API
        return self._extract_rosbag2_api(topics)

    def _extract_sqlite_direct(self, topics: List[str]) -> bool:
        """Extract data directly from sqlite3 database (bypasses metadata version issues)"""
        import sqlite3

        # Find db3 file
        db_file = self._find_db3_file()
        if not db_file:
            print("No sqlite3 database found, trying rosbag2 API...")
            return False

        print(f"Opening database directly: {db_file}")

        try:
            conn = sqlite3.connect(str(db_file))
            cursor = conn.cursor()

            # Get topic IDs
            cursor.execute("SELECT id, name FROM topics")
            topic_map = {name: tid for tid, name in cursor.fetchall()}

            # Check which topics exist
            found_topics = [t for t in topics if t in topic_map]
            if not found_topics:
                print(f"Warning: None of the requested topics found in bag")
                print(f"  Available topics: {list(topic_map.keys())}")
                conn.close()
                return False

            print(f"Found topics: {found_topics}")

            # Read messages for each topic
            for topic in found_topics:
                topic_id = topic_map[topic]
                cursor.execute(
                    "SELECT data, timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp",
                    (topic_id,)
                )

                for data_blob, timestamp in cursor.fetchall():
                    # CDR-encoded std_msgs/String: skip 4-byte header + 4-byte length
                    try:
                        json_str = self._decode_cdr_string(data_blob)
                        self._process_message(topic, json_str, timestamp)
                    except Exception as e:
                        print(f"Warning: Failed to decode message on {topic}: {e}")

            conn.close()

            print(f"\nExtraction complete:")
            print(f"  Obstacle messages: {self.message_counts['obstacles']}")
            print(f"  - Inserts: {self.message_counts['inserts']}")
            print(f"  - Deletes: {self.message_counts['deletes']}")
            print(f"  - Final obstacles: {len(self.obstacles)}")
            print(f"  Contour messages: {self.message_counts['contours']}")

            return True

        except Exception as e:
            print(f"Error reading database: {e}")
            return False

    def _decode_cdr_string(self, data: bytes) -> str:
        """Decode CDR-serialized std_msgs/String message"""
        # CDR format for std_msgs/String:
        # - 4 bytes: CDR header (encapsulation)
        # - 4 bytes: string length (uint32, little-endian)
        # - N bytes: string data (UTF-8)
        # - padding to 4-byte boundary

        if len(data) < 8:
            raise ValueError(f"Data too short: {len(data)} bytes")

        # Skip CDR header (4 bytes)
        # Read string length (4 bytes, little-endian)
        import struct
        str_len = struct.unpack('<I', data[4:8])[0]

        if len(data) < 8 + str_len:
            raise ValueError(f"Data truncated: expected {8 + str_len} bytes, got {len(data)}")

        # Read string (includes null terminator)
        string_data = data[8:8 + str_len]

        # Remove null terminator if present
        if string_data and string_data[-1] == 0:
            string_data = string_data[:-1]

        return string_data.decode('utf-8')

    def _find_db3_file(self) -> Optional[Path]:
        """Find the sqlite3 database file"""
        bag_path = Path(self.bag_path)

        if bag_path.is_file() and bag_path.suffix == '.db3':
            return bag_path

        if bag_path.is_dir():
            db_files = list(bag_path.glob('*.db3'))
            if db_files:
                return db_files[0]

        return None

    def _extract_rosbag2_api(self, topics: List[str]) -> bool:
        """Extract data using rosbag2_py API (fallback method)"""
        try:
            from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
            from rclpy.serialization import deserialize_message
            from std_msgs.msg import String
        except ImportError as e:
            print(f"Error: Required ROS2 packages not found: {e}")
            print("Please run this script in a ROS2 environment:")
            print("  source /opt/ros/humble/setup.bash")
            return False

        storage_id = self._detect_storage_type()
        if not storage_id:
            print(f"Error: Could not detect storage type for {self.bag_path}")
            return False

        print(f"Opening bag via rosbag2 API: {self.bag_path} (storage: {storage_id})")

        storage_options = StorageOptions(
            uri=str(self.bag_path),
            storage_id=storage_id
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = SequentialReader()
        try:
            reader.open(storage_options, converter_options)
        except Exception as e:
            print(f"Error opening bag: {e}")
            return False

        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if topic in topics:
                try:
                    msg = deserialize_message(data, String)
                    self._process_message(topic, msg.data, timestamp)
                except Exception as e:
                    print(f"Warning: Failed to deserialize message on {topic}: {e}")

        print(f"\nExtraction complete:")
        print(f"  Obstacle messages: {self.message_counts['obstacles']}")
        print(f"  - Inserts: {self.message_counts['inserts']}")
        print(f"  - Deletes: {self.message_counts['deletes']}")
        print(f"  - Final obstacles: {len(self.obstacles)}")
        print(f"  Contour messages: {self.message_counts['contours']}")

        return True

    def _detect_storage_type(self) -> Optional[str]:
        """Detect the rosbag storage type"""
        bag_path = Path(self.bag_path)

        if bag_path.is_file():
            if bag_path.suffix == '.db3':
                return 'sqlite3'
            if bag_path.suffix == '.mcap':
                return 'mcap'

        if bag_path.is_dir():
            if list(bag_path.glob('*.db3')):
                return 'sqlite3'
            if list(bag_path.glob('*.mcap')):
                return 'mcap'

        return 'sqlite3'  # Default

    def _process_message(self, topic: str, json_str: str, timestamp: int):
        """Process a single message"""
        try:
            data = json.loads(json_str)
        except json.JSONDecodeError as e:
            print(f"Warning: Invalid JSON on {topic}: {e}")
            return

        if topic == '/mqtt/discerning_safety_map':
            self._process_obstacles(data)
        elif topic == '/mqtt/warehouse_contours':
            self._process_contours(data)

    def _process_obstacles(self, data: Dict):
        """Process obstacle map update message"""
        self.message_counts['obstacles'] += 1

        pallets = data.get('pallets', [])
        for pallet in pallets:
            pallet_id = pallet.get('id')
            operation = pallet.get('operation', 'INSERT')
            aabb = pallet.get('aabb')

            if pallet_id is None:
                continue

            if operation == 'INSERT' and aabb:
                # aabb format: [[x_min, y_min], [x_max, y_max]]
                try:
                    self.obstacles[pallet_id] = Obstacle(
                        id=pallet_id,
                        x_min=aabb[0][0],
                        y_min=aabb[0][1],
                        x_max=aabb[1][0],
                        y_max=aabb[1][1]
                    )
                    self.message_counts['inserts'] += 1
                except (IndexError, TypeError) as e:
                    print(f"Warning: Invalid AABB format for pallet {pallet_id}: {e}")

            elif operation == 'DELETE':
                if pallet_id in self.obstacles:
                    del self.obstacles[pallet_id]
                self.message_counts['deletes'] += 1

    def _process_contours(self, data: Dict):
        """Process warehouse contours message"""
        self.message_counts['contours'] += 1

        # Parse contours - keep only the latest
        self.warehouse_contours = WarehouseContours(
            outer_contour_hull=data.get('outer_contour_hull', []),
            outer_contour_segments=data.get('outer_contour_segments', []),
            inner_contours=data.get('inner_contours', [])
        )

    def save_obstacles(self, output_path: str) -> bool:
        """Save obstacles to JSON file"""
        if not self.obstacles:
            print("Warning: No obstacles to save")
            return False

        # Convert obstacles to JSON format matching the input format
        # This allows the translator to process it the same way
        pallets = []
        for obs in self.obstacles.values():
            pallets.append({
                'id': obs.id,
                'operation': 'INSERT',
                'aabb': [[obs.x_min, obs.y_min], [obs.x_max, obs.y_max]]
            })

        output_data = {
            'pallets': pallets,
            '_metadata': {
                'extracted_from': str(self.bag_path),
                'extracted_at': datetime.now().isoformat(),
                'obstacle_count': len(pallets)
            }
        }

        try:
            with open(output_path, 'w') as f:
                json.dump(output_data, f, indent=2)
            print(f"Saved {len(pallets)} obstacles to {output_path}")
            return True
        except Exception as e:
            print(f"Error saving obstacles: {e}")
            return False

    def save_contours(self, output_path: str) -> bool:
        """Save warehouse contours to JSON file"""
        if not self.warehouse_contours:
            print("Warning: No contours to save")
            return False

        output_data = {
            'outer_contour_hull': self.warehouse_contours.outer_contour_hull,
            'outer_contour_segments': self.warehouse_contours.outer_contour_segments,
            'inner_contours': self.warehouse_contours.inner_contours,
            '_metadata': {
                'extracted_from': str(self.bag_path),
                'extracted_at': datetime.now().isoformat(),
                'hull_points': len(self.warehouse_contours.outer_contour_hull),
                'segment_count': len(self.warehouse_contours.outer_contour_segments),
                'inner_contour_count': len(self.warehouse_contours.inner_contours)
            }
        }

        try:
            with open(output_path, 'w') as f:
                json.dump(output_data, f, indent=2)
            print(f"Saved contours to {output_path}")
            return True
        except Exception as e:
            print(f"Error saving contours: {e}")
            return False

    def get_summary(self) -> Dict[str, Any]:
        """Get extraction summary"""
        summary = {
            'bag_path': str(self.bag_path),
            'obstacles': {
                'count': len(self.obstacles),
                'inserts_processed': self.message_counts['inserts'],
                'deletes_processed': self.message_counts['deletes'],
            },
            'contours': {
                'found': self.warehouse_contours is not None,
            }
        }

        if self.warehouse_contours:
            summary['contours']['hull_points'] = len(self.warehouse_contours.outer_contour_hull)
            summary['contours']['segments'] = len(self.warehouse_contours.outer_contour_segments)
            summary['contours']['inner_polygons'] = len(self.warehouse_contours.inner_contours)

        return summary


def main():
    parser = argparse.ArgumentParser(
        description='Extract warehouse map data from rosbag files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Extract all map data to default files
  %(prog)s /path/to/bagfile

  # Extract to specific output directory
  %(prog)s /path/to/bagfile -o /output/dir

  # Extract only obstacles
  %(prog)s /path/to/bagfile --obstacles-only

  # Extract only contours
  %(prog)s /path/to/bagfile --contours-only

  # Custom output filenames
  %(prog)s /path/to/bagfile --obstacles-file my_obstacles.json --contours-file my_contours.json
        """
    )

    parser.add_argument('bag_path', help='Path to rosbag file or directory')
    parser.add_argument('-o', '--output-dir',
                        help='Output directory (default: same as bag)')
    parser.add_argument('--obstacles-file',
                        help='Output filename for obstacles (default: obstacles.json)')
    parser.add_argument('--contours-file',
                        help='Output filename for contours (default: warehouse_contours.json)')
    parser.add_argument('--obstacles-only', action='store_true',
                        help='Extract only obstacles')
    parser.add_argument('--contours-only', action='store_true',
                        help='Extract only contours')
    parser.add_argument('--summary', action='store_true',
                        help='Print summary and exit without saving')

    args = parser.parse_args()

    # Validate bag path
    bag_path = Path(args.bag_path)
    if not bag_path.exists():
        print(f"Error: Bag path does not exist: {bag_path}")
        return 1

    # Determine output directory
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        output_dir = bag_path if bag_path.is_dir() else bag_path.parent

    output_dir.mkdir(parents=True, exist_ok=True)

    # Determine which topics to extract
    topics = []
    if not args.contours_only:
        topics.append('/mqtt/discerning_safety_map')
    if not args.obstacles_only:
        topics.append('/mqtt/warehouse_contours')

    # Extract data
    extractor = MapDataExtractor(str(bag_path))
    if not extractor.extract(topics):
        return 1

    # Print summary if requested
    if args.summary:
        print("\nSummary:")
        print(json.dumps(extractor.get_summary(), indent=2))
        return 0

    # Save extracted data
    success = True

    if not args.contours_only:
        obstacles_file = args.obstacles_file or 'obstacles.json'
        obstacles_path = output_dir / obstacles_file
        if not extractor.save_obstacles(str(obstacles_path)):
            success = False

    if not args.obstacles_only:
        contours_file = args.contours_file or 'warehouse_contours.json'
        contours_path = output_dir / contours_file
        if not extractor.save_contours(str(contours_path)):
            success = False

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
