# ROS2-Week-2-Creating-and-building-Packages
This repo is all about creating and building ROS2 packages
🎯 Learning Objectives
By the end of this week, you will be able to:

✅ Understand ROS 2 workspace structure

✅ Create and build ROS 2 packages (Python & C++)

✅ Master package.xml and CMakeLists.txt configuration

✅ Use colcon build system effectively

✅ Create launch files for multi-node systems

✅ Understand dependencies and package relationships

Key Directories:

src/: Your source code - where you develop packages

build/: Intermediate build files (object files, CMake cache)

install/: Final installation - where executables and libraries go

log/: Build logs, warnings, and errors

2.2 Package Types in ROS 2

1. Python Packages (ament_python)

Pure Python packages

Use setup.py for installation

Lighter weight, easier for beginners

2. C++ Packages (ament_cmake)

C/C++ packages

Use CMakeLists.txt for building

Better performance for compute-intensive tasks

3. Hybrid Packages

Can contain both Python and C++ code

More complex configuration

2.3 The Build System: Colcon

Colcon (Collective Construction)

Successor to catkin_make from ROS 1

More flexible and modern

Supports parallel building

Common colcon commands:

# Build all packages in workspace
colcon build

# Build specific package
colcon build --packages-select my_package

# Build with symlink installation (faster development)
colcon build --symlink-install

# Build in release mode
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build with debugging
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

⚙️ Hands-On Setup
Step 1: Create Your First Workspace

# Create workspace directory
mkdir -p ~/ros2_ws/src

cd ~/ros2_ws

# Source ROS 2 (if not already in .bashrc)
source /opt/ros/humble/setup.bash

# Build empty workspace (creates build/install/log)
colcon build

# Source your workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

source ~/.bashrc

Step 2: Understanding Package Creation

Python Package Structure:

🔧 Practical Exercises
Exercise 1: Create Your First Python Package

cd ~/ros2_ws/src

# Create a Python package
    ros2 pkg create my_first_py_pkg --build-type ament_python \

    --dependencies rclpy std_msgs \
    
    --description "My first ROS 2 Python package" \
    
    --license "Apache-2.0"

# Explore the created structure
tree my_first_py_pkg

Generated package.xml:

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
      <name>my_first_py_pkg</name>
      <version>0.0.0</version>
      <description>My first ROS 2 Python package</description>
      <maintainer email="user@todo.todo">user</maintainer>
      <license>Apache-2.0</license>

      <depend>rclpy</depend>
      <depend>std_msgs</depend>

      <test_depend>ament_copyright</test_depend>
      <test_depend>ament_flake8</test_depend>
      <test_depend>ament_pep257</test_depend>
      <test_depend>python3-pytest</test_depend>

      <export>
        <build_type>ament_python</build_type>
      </export>
    </package>

Exercise 2: Create Your First C++ Package

    cd ~/ros2_ws/src

# Create a C++ package
    ros2 pkg create my_first_cpp_pkg --build-type ament_cmake \
    --dependencies rclcpp std_msgs \
    --description "My first ROS 2 C++ package" \
    --license "Apache-2.0"

# Explore the structure
tree my_first_cpp_pkg

    Generated CMakeLists.txt (partial):

    cmake_minimum_required(VERSION 3.8)

    project(my_first_cpp_pkg)

# Default to C++17

    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 17)
    endif()

# Find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

# Add executables
    add_executable(my_node src/my_node.cpp)
    ament_target_dependencies(my_node rclcpp std_msgs)

# Install executables
    install(TARGETS my_node
      DESTINATION lib/${PROJECT_NAME})

# Export dependencies
    ament_export_dependencies(rclcpp std_msgs)
    ament_package()

Exercise 3: Add Custom Code to Python Package

1. Update the Python module structure:

        cd ~/ros2_ws/src/my_first_py_pkg
        mkdir -p my_first_py_pkg
2. Create a simple publisher node (my_first_py_pkg/simple_publisher.py):

        #!/usr/bin/env python3
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        import time

        class SimplePublisher(Node):
            def __init__(self):
                super().__init__('simple_publisher')
                self.publisher = self.create_publisher(String, 'my_topic', 10)
                timer_period = 1.0
                self.timer = self.create_timer(timer_period, self.timer_callback)
                self.counter = 0
        
            def timer_callback(self):
                msg = String()
                msg.data = f'Message #{self.counter} from Python package'
                self.publisher.publish(msg)
                self.get_logger().info(f'Published: "{msg.data}"')
                self.counter += 1

            def main(args=None):
                rclpy.init(args=args)
                node = SimplePublisher()
    
                try:
                    rclpy.spin(node)
                except KeyboardInterrupt:
                    pass
    
                node.destroy_node()
                rclpy.shutdown()

       if __name__ == '__main__':
            main()
3. Update setup.py to include the executable:

        from setuptools import setup
        package_name = 'my_first_py_pkg'

        setup(
            name=package_name,
            version='0.0.0',
            packages=[package_name],
            data_files=[
                ('share/ament_index/resource_index/packages',
                    ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
            ],
            install_requires=['setuptools'],
            zip_safe=True,
            maintainer='your_name',
            maintainer_email='your_email@example.com',
            description='My first ROS 2 Python package',
            license='Apache-2.0',
            tests_require=['pytest'],
            entry_points={
                'console_scripts': [
                    'simple_publisher = my_first_py_pkg.simple_publisher:main',
                ],
            },
       )
Exercise 4: Add Custom Code to C++ Package
1. Create C++ node (my_first_cpp_pkg/src/simple_subscriber.cpp):

        #include "rclcpp/rclcpp.hpp"
        #include "std_msgs/msg/string.hpp"

        using std::placeholders::_1;

        class SimpleSubscriber : public rclcpp::Node
        {
        public:
          SimpleSubscriber() : Node("simple_subscriber")
          {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
              "my_topic", 10, std::bind(&SimpleSubscriber::topic_callback, this, _1));
    
            RCLCPP_INFO(this->get_logger(), "C++ Subscriber started. Waiting for messages...");
          }

        private:
          void topic_callback(const std_msgs::msg::String::SharedPtr msg)
          {
            RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
          }
  
          rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        };

        int main(int argc, char * argv[])
        {
          rclcpp::init(argc, argv);
          rclcpp::spin(std::make_shared<SimpleSubscriber>());
          rclcpp::shutdown();
          return 0;
        }
2. Update CMakeLists.txt to add the executable:

# Add after find_package calls
    add_executable(simple_subscriber src/simple_subscriber.cpp)
    ament_target_dependencies(simple_subscriber rclcpp std_msgs)

# Add to install section
    install(TARGETS simple_subscriber
      DESTINATION lib/${PROJECT_NAME})
Exercise 5: Build and Run
# Build all packages
    cd ~/ros2_ws
    colcon build --symlink-install

# Source the workspace
    source install/setup.bash

# Terminal 1: Run Python publisher
    ros2 run my_first_py_pkg simple_publisher

# Terminal 2: Run C++ subscriber
    ros2 run my_first_cpp_pkg simple_subscriber

💻 Advanced Package Concepts

2.4 Creating Custom Messages

Step 1: Create message package

    cd ~/ros2_ws/src
    ros2 pkg create my_custom_msgs --build-type ament_cmake \
        --dependencies std_msgs geometry_msgs

# Create message directory
    mkdir -p my_custom_msgs/msg
       
