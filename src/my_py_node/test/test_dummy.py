import pytest
import rclpy
from rclpy.node import Node


@pytest.fixture(scope='module')
def test_node():
    rclpy.init()
    node = Node('test_node_py')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_node_name(test_node):
    assert test_node.get_name() == 'test_node_py'


def test_addition():
    assert 3 + 4 == 7
