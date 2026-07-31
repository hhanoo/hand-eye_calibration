"""
Tests for pose staleness detection.

Both readers can serve an outdated pose without raising: the UR reader
when TCP chunks are mis-framed, the TF reader from its multi-second
cache after the publisher stops. A stale pose paired with a fresh camera
image is what corrupted the 108-sample dataset, so each reader exposes a
pose_seq counter that only advances on genuinely new data.
"""

import struct

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node

from hand_eye_calibration.robot_interface import ROS2TFPoseReader, URDirectPoseReader

PACKET_SIZE = 1108
POSE_OFFSET = 444


def make_packet(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0, size=PACKET_SIZE):
    """Build one UR real-time packet carrying the given TCP pose."""
    body = bytearray(size)
    body[0:4] = struct.pack(">i", size)
    body[POSE_OFFSET : POSE_OFFSET + 48] = struct.pack(">6d", x, y, z, rx, ry, rz)
    return bytes(body)


def reader():
    return URDirectPoseReader("192.0.2.1")


def test_single_complete_packet_is_parsed():
    r = reader()

    r._feed(make_packet(x=0.1, y=0.2, z=0.3))

    assert np.allclose(r.get_pose()[:3, 3], [0.1, 0.2, 0.3])


def test_uses_most_recent_packet_when_several_arrive_together():
    r = reader()
    chunk = make_packet(x=1.0) + make_packet(x=2.0) + make_packet(x=3.0)

    r._feed(chunk)

    assert r.get_pose()[0, 3] == pytest.approx(3.0)


def test_packet_split_across_reads_is_reassembled():
    r = reader()
    packet = make_packet(x=0.5)

    r._feed(packet[:600])
    with pytest.raises(RuntimeError):
        r.get_pose()

    r._feed(packet[600:])
    assert r.get_pose()[0, 3] == pytest.approx(0.5)


def test_resynchronizes_after_misaligned_start():
    r = reader()

    # Stream joined mid-packet: leading bytes are the tail of an older packet.
    r._feed(b"\x11" * 137 + make_packet(x=0.7))

    assert r.get_pose()[0, 3] == pytest.approx(0.7)


def test_sequence_advances_only_when_a_new_packet_arrives():
    r = reader()
    packet = make_packet(x=1.0)

    r._feed(packet)
    first = r.pose_seq

    r._feed(packet[:500])  # incomplete: nothing new to report
    assert r.pose_seq == first

    r._feed(packet[500:])
    assert r.pose_seq == first + 1


def test_rotation_vector_is_applied():
    r = reader()

    r._feed(make_packet(rz=np.pi / 2))

    R = r.get_pose()[:3, :3]
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)


def test_buffer_does_not_grow_without_bound():
    r = reader()

    for _ in range(50):
        r._feed(make_packet())

    assert len(r._buffer) < 2 * PACKET_SIZE


def test_a_motionless_robot_still_advances_the_sequence():
    # The pose is identical every cycle, but the stream is live — this must
    # stay distinguishable from a stalled stream reporting a frozen pose.
    r = reader()
    still = make_packet(x=0.4, y=0.1, z=0.2)

    r._feed(still)
    first = r.pose_seq
    r._feed(still)

    assert r.pose_seq == first + 1
    assert np.allclose(r.get_pose()[:3, 3], [0.4, 0.1, 0.2])


def test_arbitrarily_chunked_stream_tracks_the_newest_pose():
    # Reproduces the acquisition failure: a real socket splits and coalesces
    # packets at arbitrary offsets. Every chunk boundary must still leave
    # get_pose() reporting the most recent pose the controller sent.
    rng = np.random.default_rng(0)
    r = reader()
    stream = b"".join(make_packet(x=float(i)) for i in range(40))

    pos = 0
    while pos < len(stream):
        chunk = int(rng.integers(1, 3000))
        r._feed(stream[pos : pos + chunk])
        pos += chunk

    assert r.get_pose()[0, 3] == pytest.approx(39.0)


def test_pose_is_never_older_than_the_last_complete_packet():
    r = reader()
    stream = b"".join(make_packet(x=float(i)) for i in range(10))

    # Deliver everything except the tail of the final packet.
    r._feed(stream[: -PACKET_SIZE // 2])

    # The newest fully received packet is index 8, not the first of the batch.
    assert r.get_pose()[0, 3] == pytest.approx(8.0)


@pytest.fixture
def node():
    rclpy.init()
    n = Node("test_pose_reader")
    yield n
    n.destroy_node()
    rclpy.shutdown()


def make_tf(stamp_sec, x=0.0):
    t = TransformStamped()
    t.header.stamp.sec = int(stamp_sec)
    t.header.stamp.nanosec = int((stamp_sec % 1) * 1e9)
    t.header.frame_id = "base_link"
    t.child_frame_id = "tool0"
    t.transform.translation.x = x
    t.transform.rotation.w = 1.0
    return t


def test_tf_sequence_advances_on_a_newly_stamped_transform(node):
    r = ROS2TFPoseReader(node, "base_link", "tool0")
    r.tf_buffer.set_transform(make_tf(1.0, x=0.3), "test")
    r.get_pose()
    first = r.pose_seq

    # Same pose, newer stamp: robot is motionless but the stream is live.
    r.tf_buffer.set_transform(make_tf(1.5, x=0.3), "test")
    pose = r.get_pose()

    assert r.pose_seq == first + 1
    assert pose[0, 3] == pytest.approx(0.3)


def test_tf_sequence_frozen_when_the_publisher_stops(node):
    r = ROS2TFPoseReader(node, "base_link", "tool0")
    r.tf_buffer.set_transform(make_tf(1.0, x=0.1), "test")
    r.get_pose()
    first = r.pose_seq

    # No new transform: the buffer still answers from its cache.
    r.get_pose()
    r.get_pose()

    assert r.pose_seq == first
