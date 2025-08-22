#include "ros2mapbridge.h"
#include "mapimageprovider.h"
#include <QPainter>
#include <QDebug>
#include <tf2/utils.h>
#include <cmath>

Ros2MapBridge::Ros2MapBridge(QObject *parent)
    : QObject(parent)
    , m_robotX(0.0)
    , m_robotY(0.0)
    , m_robotTheta(0.0)
    , m_isConnected(false)
    , m_mapResolution(0.05)
    , m_mapOriginX(0.0)
    , m_mapOriginY(0.0)
    , m_mapWidth(0)
    , m_mapHeight(0)
    , m_imageProvider(nullptr)
    , m_imageCounter(0)
    , m_currentLinearVelocity(0.0)
    , m_currentAngularVelocity(0.0)
    , m_isMoving(false)
{
    // Initialize timers
    m_spinTimer = new QTimer(this);
    connect(m_spinTimer, &QTimer::timeout, this, &Ros2MapBridge::spinOnce);
    
    m_poseUpdateTimer = new QTimer(this);
    connect(m_poseUpdateTimer, &QTimer::timeout, this, &Ros2MapBridge::updateRobotPose);
    
    // Initialize velocity publishing timer at 50 Hz
    m_velocityPublishTimer = new QTimer(this);
    connect(m_velocityPublishTimer, &QTimer::timeout, this, &Ros2MapBridge::publishCurrentVelocity);
    
    // Initialize with empty map URL
    m_mapImageUrl = "image://mapprovider/map";
}

Ros2MapBridge::~Ros2MapBridge()
{
    disconnectFromRos2();
}

void Ros2MapBridge::connectToRos2()
{
    try {
        // Initialize ROS2 if not already done
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // Create ROS2 node
        m_node = std::make_shared<rclcpp::Node>("robot_face_map_bridge");
        
        // Subscribe to map topic
        m_mapSubscriber = m_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                this->mapCallback(msg);
            });
        
        // Publisher for navigation goals
        m_goalPublisher = m_node->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);
        
        // Publisher for velocity commands
        m_cmdVelPublisher = m_node->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        // Initialize TF2
        m_tfBuffer = std::make_shared<tf2_ros::Buffer>(m_node->get_clock());
        m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
        
        // Start timers
        m_spinTimer->start(50); // 20 Hz
        m_poseUpdateTimer->start(100); // 10 Hz
        m_velocityPublishTimer->start(20); // 50 Hz
        
        m_isConnected = true;
        emit connectionStatusChanged();
        
        qDebug() << "Connected to ROS2 successfully";
        
    } catch (const std::exception& e) {
        qDebug() << "Failed to connect to ROS2:" << e.what();
        m_isConnected = false;
        emit connectionStatusChanged();
    }
}

void Ros2MapBridge::disconnectFromRos2()
{
    m_spinTimer->stop();
    m_poseUpdateTimer->stop();
    m_velocityPublishTimer->stop();
    
    m_mapSubscriber.reset();
    m_goalPublisher.reset();
    m_cmdVelPublisher.reset();
    m_tfListener.reset();
    m_tfBuffer.reset();
    m_node.reset();
    
    m_isConnected = false;
    emit connectionStatusChanged();
    
    qDebug() << "Disconnected from ROS2";
}

void Ros2MapBridge::spinOnce()
{
    if (m_node && rclcpp::ok()) {
        rclcpp::spin_some(m_node);
    }
}

void Ros2MapBridge::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (!msg) return;
    
    m_currentMap = msg;
    m_mapWidth = msg->info.width;
    m_mapHeight = msg->info.height;
    m_mapResolution = msg->info.resolution;
    m_mapOriginX = msg->info.origin.position.x;
    m_mapOriginY = msg->info.origin.position.y;
    
    QImage mapImage = convertMapToImage(msg);
    updateMapImage(mapImage);
    updateMapInfo();
    
    emit mapInfoChanged();
}

QImage Ros2MapBridge::convertMapToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (!msg || msg->data.empty()) {
        return QImage();
    }
    
    QImage image(msg->info.width, msg->info.height, QImage::Format_RGB32);
    
    for (unsigned int y = 0; y < msg->info.height; ++y) {
        for (unsigned int x = 0; x < msg->info.width; ++x) {
            unsigned int index = y * msg->info.width + x;
            int8_t value = msg->data[index];
            
            QColor color;
            if (value == -1) {
                // Unknown
                color = QColor(128, 128, 128);
            } else if (value == 0) {
                // Free space
                color = QColor(255, 255, 255);
            } else {
                // Occupied - gradient based on probability
                int gray = 255 - (value * 255 / 100);
                color = QColor(gray, gray, gray);
            }
            
            // Set pixel (flip y-axis for correct orientation)
            image.setPixelColor(x, msg->info.height - 1 - y, color);
        }
    }
    
    return image;
}

void Ros2MapBridge::updateMapImage(const QImage& image)
{
    if (m_imageProvider) {
        m_imageProvider->updateImage(image);
        
        // Update the URL with a counter to force refresh in QML
        m_imageCounter++;
        m_mapImageUrl = QString("image://mapprovider/map?id=%1").arg(m_imageCounter);
        emit mapImageChanged();
    }
}

void Ros2MapBridge::updateRobotPose()
{
    if (!m_tfBuffer) return;
    
    try {
        geometry_msgs::msg::TransformStamped transform;
        transform = m_tfBuffer->lookupTransform("map", "base_link", 
                                                tf2::TimePointZero);
        
        m_robotX = transform.transform.translation.x;
        m_robotY = transform.transform.translation.y;
        
        // Calculate theta from quaternion
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        m_robotTheta = tf2::getYaw(q);
        
        emit robotPoseChanged();
        
    } catch (const tf2::TransformException& ex) {
        // Transform not available yet, this is normal at startup
    }
}

void Ros2MapBridge::setGoalPose(double x, double y)
{
    if (!m_goalPublisher) return;
    
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = m_node->now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;
    
    // Simple orientation towards goal from current position
    double dx = x - m_robotX;
    double dy = y - m_robotY;
    double theta = std::atan2(dy, dx);
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();
    
    m_goalPublisher->publish(goal);
    
    emit navigationStatus(QString("Goal set to (%1, %2)").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
}

void Ros2MapBridge::clearGoalPose()
{
    emit navigationStatus("Goal cleared");
}

QPointF Ros2MapBridge::mapToScreen(double mapX, double mapY)
{
    if (m_mapWidth == 0 || m_mapHeight == 0) {
        return QPointF(0, 0);
    }
    
    // Convert from map coordinates to image pixels
    double pixelX = (mapX - m_mapOriginX) / m_mapResolution;
    double pixelY = (mapY - m_mapOriginY) / m_mapResolution;
    
    // Flip Y axis for screen coordinates
    pixelY = m_mapHeight - pixelY;
    
    return QPointF(pixelX, pixelY);
}

QPointF Ros2MapBridge::screenToMap(double screenX, double screenY)
{
    // Flip Y axis from screen coordinates
    double pixelY = m_mapHeight - screenY;
    
    // Convert from image pixels to map coordinates
    double mapX = screenX * m_mapResolution + m_mapOriginX;
    double mapY = pixelY * m_mapResolution + m_mapOriginY;
    
    return QPointF(mapX, mapY);
}

void Ros2MapBridge::updateMapInfo()
{
    if (!m_currentMap) {
        m_mapInfo = "No map data available";
        return;
    }
    
    m_mapInfo = QString("Map: %1x%2 pixels, Resolution: %3 m/pixel")
                .arg(m_mapWidth)
                .arg(m_mapHeight)
                .arg(m_mapResolution, 0, 'f', 3);
}

void Ros2MapBridge::sendVelocity(double linear, double angular)
{
    if (!m_cmdVelPublisher) {
        qDebug() << "cmd_vel publisher not available";
        return;
    }
    
    // Store current velocities for continuous publishing
    m_currentLinearVelocity = linear;
    m_currentAngularVelocity = angular;
    m_isMoving = (linear != 0.0 || angular != 0.0);
    
    // Publish immediately
    publishCurrentVelocity();
    
    // Debug output for non-zero velocities
    if (m_isMoving) {
        qDebug() << "Setting velocity - Linear:" << linear << "Angular:" << angular;
    }
}

void Ros2MapBridge::cancelGoal()
{
    // For now, we'll just clear the goal
    // In a full implementation, you might want to use Navigation2 action client to cancel
    clearGoalPose();
    emit navigationStatus("Goal cancelled");
}

void Ros2MapBridge::stopVelocity()
{
    // Stop all movement and clear velocity state
    m_currentLinearVelocity = 0.0;
    m_currentAngularVelocity = 0.0;
    m_isMoving = false;
    
    // Send one final stop command
    if (m_cmdVelPublisher) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
        
        m_cmdVelPublisher->publish(cmd_vel);
        qDebug() << "Stopping robot - sending zero velocity command";
    }
}

void Ros2MapBridge::publishCurrentVelocity()
{
    if (!m_cmdVelPublisher) {
        return;
    }
    
    // Only publish if we're actually moving (non-zero velocities)
    if (!m_isMoving) {
        return;
    }
    
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = m_currentLinearVelocity;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = m_currentAngularVelocity;
    
    m_cmdVelPublisher->publish(cmd_vel);
    
    // Debug output for non-zero velocities (less frequent to avoid spam)
    static int debugCounter = 0;
    if (++debugCounter % 50 == 0) { // Log every 1 second at 50 Hz
        qDebug() << "Publishing cmd_vel - Linear:" << m_currentLinearVelocity << "Angular:" << m_currentAngularVelocity;
    }
}
