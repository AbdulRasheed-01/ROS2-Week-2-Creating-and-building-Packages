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
       
Step 2: Create custom message (my_custom_msgs/msg/VehicleStatus.msg)

# Vehicle status message
    std_msgs/Header header
    string vehicle_id
    float32 speed           # km/h
    float32 battery_level   # percentage
    float32 temperature     # °C
    bool engine_on
    geometry_msgs/Pose pose

Step 3: Update CMakeLists.txt
# Add after find_package
    find_package(rosidl_default_generators REQUIRED)

# Add message generation
    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/VehicleStatus.msg"
      DEPENDENCIES std_msgs geometry_msgs
    )
Step 4: Update package.xml

    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>

2.5 Creating Launch Files
Python Launch File (my_first_py_pkg/launch/demo.launch.py):

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='my_first_py_pkg',
                executable='simple_publisher',
                name='my_publisher',
                output='screen',
                parameters=[{'publish_rate': 2.0}]
            ),
            Node(
                package='my_first_cpp_pkg',
                executable='simple_subscriber',
                name='my_subscriber',
                output='screen'
            ),
        ])
Run the launch file:

    ros2 launch my_first_py_pkg demo.launch.py
2.6 Package Dependencies
Types of dependencies in package.xml:

    <!-- Build dependencies (needed during compilation) -->
    <build_depend>package_name</build_depend>

    <!-- Execution dependencies (needed at runtime) -->
    <exec_depend>package_name</exec_depend>

    <!-- Test dependencies (needed for testing) -->
    <test_depend>package_name</test_depend>

    <!-- Shortcut: depend = build_depend + exec_depend -->
    <depend>package_name</depend>
📝 Weekly Project: Multi-Robot Communication System
Project Description
Create a package that simulates a multi-robot system with:

Robot Controller Package (Python) - Controls robot movement

Sensor Package (C++) - Simulates sensor data

Monitor Package (Python) - Monitors all robots

Custom Messages - For robot status

Launch Files - To start the entire system

Step-by-Step Implementation

Step 1: Create Workspace Structure

    cd ~/ros2_ws/src
# Create packages
    ros2 pkg create robot_controller --build-type ament_python \
        --dependencies rclpy geometry_msgs

    ros2 pkg create robot_sensors --build-type ament_cmake \
        --dependencies rclcpp sensor_msgs

    ros2 pkg create system_monitor --build-type ament_python \
        --dependencies rclpy std_msgs

    ros2 pkg create robot_msgs --build-type ament_cmake \
        --dependencies std_msgs geometry_msgs
Step 2: Create Custom Messages

Create robot_msgs/msg/RobotStatus.msg:

    string robot_id
    geometry_msgs/Pose pose
    float32 battery_level
    float32 velocity
    string status  # "IDLE", "MOVING", "CHARGING"

Step 3: Implement Robot Controller

robot_controller/robot_controller/simple_robot.py:

    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, Pose
    from robot_msgs.msg import RobotStatus
    import random

    class SimpleRobot(Node):
        def __init__(self, robot_id):
            super().__init__(f'robot_{robot_id}')
            self.robot_id = robot_id
        
        # Publisher for robot status
            self.status_pub = self.create_publisher(
            RobotStatus, f'/robot/{robot_id}/status', 10)
        
        # Subscriber for velocity commands
            self.cmd_sub = self.create_subscription(
            Twist, f'/robot/{robot_id}/cmd_vel', self.cmd_callback, 10)
        
        # Timer for periodic status updates
            self.timer = self.create_timer(1.0, self.publish_status)
        
        # Robot state
            self.pose = Pose()
            self.battery = 100.0
            self.velocity = 0.0
            self.status = "IDLE"
        
        def cmd_callback(self, msg):
            self.velocity = msg.linear.x
            self.status = "MOVING" if abs(self.velocity) > 0.1 else "IDLE"
        
        def publish_status(self):
            # Simulate battery drain
            self.battery = max(0.0, self.battery - 0.1)
        
        # Update pose based on velocity
            self.pose.position.x += self.velocity * 0.1
        
        # Create status message
            status_msg = RobotStatus()
            status_msg.robot_id = self.robot_id
            status_msg.pose = self.pose
            status_msg.battery_level = self.battery
            status_msg.velocity = self.velocity
            status_msg.status = self.status
        
            self.status_pub.publish(status_msg)
            self.get_logger().info(f'Robot {self.robot_id}: {self.status} '
                              f'Battery: {self.battery:.1f}%')
Step 4: Implement Sensor Package

robot_sensors/src/laser_scanner.cpp:
    
    #include "rclcpp/rclcpp.hpp"
    #include "sensor_msgs/msg/laser_scan.hpp"
    #include <random>

    class LaserScanner : public rclcpp::Node
    {
    public:
      LaserScanner() : Node("laser_scanner")
      {
    // Create publisher for laser scan
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
          "/scan", 10);
    
    // Timer for publishing scans (10 Hz)
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(100),
          std::bind(&LaserScanner::publish_scan, this));
    
        RCLCPP_INFO(this->get_logger(), "Laser Scanner node started");
      }

    private:
      void publish_scan()
      {
        auto scan_msg = sensor_msgs::msg::LaserScan();
        scan_msg.header.stamp = this->now();
        scan_msg.header.frame_id = "laser_frame";
    
    // Configure scan parameters
        scan_msg.angle_min = -3.14159 / 2;
        scan_msg.angle_max = 3.14159 / 2;
        scan_msg.angle_increment = 3.14159 / 180;
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 0.1;
        scan_msg.range_min = 0.1;
        scan_msg.range_max = 10.0;
    
    // Generate random ranges
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.5, 5.0);
    
        int num_readings = (scan_msg.angle_max - scan_msg.angle_min) / 
                           scan_msg.angle_increment;
    
        scan_msg.ranges.resize(num_readings);
        for (int i = 0; i < num_readings; ++i) {
          scan_msg.ranges[i] = dis(gen);
        }
    
        laser_pub_->publish(scan_msg);
      }
  
      rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
      rclcpp::TimerBase::SharedPtr timer_;
    };

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<LaserScanner>());
      rclcpp::shutdown();
      return 0;
    }
Step 5: Implement System Monitor

system_monitor/system_monitor/central_monitor.py:

    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from robot_msgs.msg import RobotStatus
    from std_msgs.msg import String
    import json

    class CentralMonitor(Node):
        def __init__(self):
            super().__init__('central_monitor')
        
        # Subscribe to all robot status topics
            self.robot_status = {}
        
        # Create subscription for each robot
            for robot_id in ['robot_1', 'robot_2', 'robot_3']:
                self.create_subscription(
                    RobotStatus,
                    f'/robot/{robot_id}/status',
                    lambda msg, rid=robot_id: self.robot_status_callback(msg, rid),
                    10
                )
        
        # Timer for system status display
            self.timer = self.create_timer(2.0, self.display_system_status)
        
            self.get_logger().info("Central Monitor started")
    
        def robot_status_callback(self, msg, robot_id):
            self.robot_status[robot_id] = {
                'battery': msg.battery_level,
                'status': msg.status,
                'position': [msg.pose.position.x, msg.pose.position.y],
                'velocity': msg.velocity
            }
    
        def display_system_status(self):
            if not self.robot_status:
                self.get_logger().info("No robots detected")
                return
        
            print("\n" + "="*50)
            print("SYSTEM STATUS MONITOR")
            print("="*50)
        
            for robot_id, status in self.robot_status.items():
                print(f"\n{robot_id}:")
                print(f"  Status: {status['status']}")
                print(f"  Battery: {status['battery']:.1f}%")
                print(f"  Position: ({status['position'][0]:.2f}, {status['position'][1]:.2f})")
                print(f"  Velocity: {status['velocity']:.2f} m/s")
        
        # Check for low battery
            low_battery = [rid for rid, s in self.robot_status.items() 
                      if s['battery'] < 20.0]
            if low_battery:
            print(f"\n⚠️  WARNING: Low battery on {', '.join(low_battery)}")
        
            print("="*50)

Step 6: Create Launch Files

robot_controller/launch/multi_robot.launch.py:

    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    def generate_launch_description():
        # Launch argument for number of robots
        num_robots_arg = DeclareLaunchArgument(
            'num_robots',
            default_value='3',
            description='Number of robots to spawn'
        )
    
        num_robots = LaunchConfiguration('num_robots')
    
        # Create robot nodes
        robot_nodes = []
        for i in range(1, 4):  # Robots 1-3
            robot_nodes.append(
                Node(
                    package='robot_controller',
                    executable='simple_robot',
                    name=f'robot_{i}',
                    parameters=[{'robot_id': str(i)}],
                    output='screen'
                )
            )
    
        # Add sensor node
        sensor_node = Node(
            package='robot_sensors',
            executable='laser_scanner',
            name='laser_scanner',
            output='screen'
        )
    
        # Add monitor node
        monitor_node = Node(
            package='system_monitor',
            executable='central_monitor',
            name='central_monitor',
            output='screen'
        )
    
        return LaunchDescription([
            num_robots_arg,
            *robot_nodes,
            sensor_node,
            monitor_node
        ])
Step 7: Build and Run
# Build all packages
    cd ~/ros2_ws
    colcon build --symlink-install
    source install/setup.bash

# Launch the entire system
    ros2 launch robot_controller multi_robot.launch.py num_robots:=3

# In another terminal, send commands to robots
    ros2 topic pub /robot/1/cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"

# Monitor topics
    ros2 topic list | grep robot
    ros2 topic echo /robot/1/status
🔍 Troubleshooting

Issue 1: "Package not found after build"
# Source your workspace
    source ~/ros2_ws/install/setup.bash

# Check if package is found
    ros2 pkg list | grep your_package

Issue 2: "Could not find a package configuration file"
# Clean and rebuild
    cd ~/ros2_ws
    rm -rf build install log
    colcon build --symlink-install
Issue 3: "ModuleNotFoundError" in Python package

    #Check setup.py entry points
    #Make sure you have:
    #entry_points={
        #'console_scripts': [
             #'node_name = package.module:main',
    #     ],    
    # }

# Reinstall the package
    cd ~/ros2_ws
    colcon build --packages-select your_package

Issue 4: "Undefined reference" in C++ package

    # Check CMakeLists.txt for missing dependencies
    # Ensure all libraries are linked:
    # ament_target_dependencies(your_node DEPENDENCIES_HERE)

    # Clean rebuild
    colcon build --cmake-clean-cache

Official Documentation:

Creating a Package

Colcon Tutorial

Custom Messages


🎉 Congratulations!
You've completed Week 2! You now know how to:

✅ Create and structure ROS 2 workspaces

✅ Build packages with colcon

✅ Create both Python and C++ packages

✅ Work with package.xml and CMakeLists.txt

✅ Create custom messages and launch files

✅ Manage package dependencies

Next Week: We'll dive into ROS 2 communication patterns - Topics, Services, and Actions!
