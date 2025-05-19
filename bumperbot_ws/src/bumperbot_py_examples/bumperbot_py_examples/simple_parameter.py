import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")

        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_string_param", "voss")

        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangeCallback(self, params):
        result = SetParametersResult()
        result.successful = True  # Assume success unless we find a problem

        for param in params:
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("simple_int_param changed to %d" % param.value)

            elif param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("simple_string_param changed to %s" % param.value)

            else:
                result.successful = False
                result.reason = f"Invalid parameter: {param.name}"

        return result

def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()