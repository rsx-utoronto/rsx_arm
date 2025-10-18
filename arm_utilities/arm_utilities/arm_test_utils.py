from rclpy.node import Node

MESSAGE_WAIT = 0.2


class test_node(Node):
    def __init__(self, publishers, subscribers):
        # input publishers and subscribers as (name, type, topic)

        super().__init__('test_node')
        self.test_publishers = {}
        for publisher in publishers:
            self.test_publishers[publisher[0]] = self.create_publisher(
                publisher[1], publisher[2], 10)

        self.test_subscribers = {}
        self.subscriber_data = {}
        for subscriber in subscribers:
            self.test_subscribers[subscriber[0]] = self.create_subscription(
                subscriber[1], subscriber[2], lambda msg, t=subscriber[2]: self.listener_callback(msg, t), 10)

    def listener_callback(self, msg, topic):
        self.subscriber_data[topic] = msg.data
