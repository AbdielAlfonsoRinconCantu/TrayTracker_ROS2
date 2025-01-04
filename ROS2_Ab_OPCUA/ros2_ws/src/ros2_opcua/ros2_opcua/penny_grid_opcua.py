import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import asyncio
from asyncua import Client, ua

PLC_url = "opc.tcp://192.168.0.114:4840"

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('penny_grid_OPCUA_server')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'penny_grid',
            self.listener_callback,
            1
        )

    def listener_callback(self, msg):
        penny_grid = msg.data
        self.get_logger().info(f"I heard: {penny_grid}")
        asyncio.run(self.update_plc(penny_grid))

    async def update_plc(self, penny_grid):
        async with Client(url=PLC_url) as client:
            for i, val in enumerate(penny_grid):
                path = f"0:Objects/2:DeviceSet/4:CODESYS Control Win V3 x64/3:Resources/4:Application/3:Programs/4:PLC_PRG/4:section{i+1}"
                plc_var = await client.nodes.root.get_child(path)
                await plc_var.write_value(bool(val))

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

