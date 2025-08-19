#include "mainwindow.h"
#include <iostream>
#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <chrono>
#include <thread>

// Callback function to handle received messages
void messageCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::cout << "Received message: " << msg->data << std::endl;
}

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    std::cout << "ROS2 node initialized!" << std::endl;
    
    // Create a ROS2 node
    auto node = std::make_shared<rclcpp::Node>("robot_app_node");
    
    // Create a subscription to a sample topic
    auto subscription = node->create_subscription<std_msgs::msg::String>(
        "sample_topic",                // Topic name
        10,                            // QoS profile (queue size)
        &messageCallback               // Callback function
    );
    
    std::cout << "Subscribed to 'sample_topic'. Waiting for messages..." << std::endl;
    
    // Create Qt application
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    
    // Create a QTimer to process ROS2 callbacks
    QTimer rosTimer;
    rosTimer.setInterval(100); // Process ROS2 messages every 100ms
    
    // Connect the timer to a lambda function that processes ROS2 callbacks
    QObject::connect(&rosTimer, &QTimer::timeout, [&node]() {
        if (rclcpp::ok()) {
            rclcpp::spin_some(node);
            std::cout << "Processing ROS2 messages..." << std::endl;
        }
    });
    
    // Start the timer
    rosTimer.start();
    
    // Run the Qt event loop
    int result = a.exec();
    
    // Clean up ROS2 resources when application exits
    rclcpp::shutdown();
    
    return result;
}
