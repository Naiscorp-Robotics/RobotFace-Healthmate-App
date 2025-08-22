#ifndef ROS2MAPBRIDGE_H
#define ROS2MAPBRIDGE_H

#include <QObject>
#include <QImage>
#include <QTimer>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class MapImageProvider;

class Ros2MapBridge : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString mapImageUrl READ mapImageUrl NOTIFY mapImageChanged)
    Q_PROPERTY(double robotX READ robotX NOTIFY robotPoseChanged)
    Q_PROPERTY(double robotY READ robotY NOTIFY robotPoseChanged)
    Q_PROPERTY(double robotTheta READ robotTheta NOTIFY robotPoseChanged)
    Q_PROPERTY(bool isConnected READ isConnected NOTIFY connectionStatusChanged)
    Q_PROPERTY(QString mapInfo READ mapInfo NOTIFY mapInfoChanged)

public:
    explicit Ros2MapBridge(QObject *parent = nullptr);
    ~Ros2MapBridge();

    Q_INVOKABLE void connectToRos2();
    Q_INVOKABLE void disconnectFromRos2();
    Q_INVOKABLE void setGoalPose(double x, double y);
    Q_INVOKABLE void clearGoalPose();
    Q_INVOKABLE void cancelGoal();
    Q_INVOKABLE void sendVelocity(double linear, double angular);
    Q_INVOKABLE void stopVelocity();
    Q_INVOKABLE QPointF mapToScreen(double mapX, double mapY);
    Q_INVOKABLE QPointF screenToMap(double screenX, double screenY);
    
    void setImageProvider(MapImageProvider* provider) { m_imageProvider = provider; }
    MapImageProvider* imageProvider() { return m_imageProvider; }

    QString mapImageUrl() const { return m_mapImageUrl; }
    double robotX() const { return m_robotX; }
    double robotY() const { return m_robotY; }
    double robotTheta() const { return m_robotTheta; }
    bool isConnected() const { return m_isConnected; }
    QString mapInfo() const { return m_mapInfo; }

signals:
    void mapImageChanged();
    void robotPoseChanged();
    void connectionStatusChanged();
    void mapInfoChanged();
    void goalReached();
    void navigationStatus(const QString &status);

private slots:
    void spinOnce();
    void updateRobotPose();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    QImage convertMapToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void updateMapInfo();
    void updateMapImage(const QImage& image);
    void publishCurrentVelocity();

    // ROS2 node and subscribers
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_mapSubscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_goalPublisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPublisher;
    
    // TF2 for robot pose
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    
    // Map data
    QString m_mapImageUrl;
    nav_msgs::msg::OccupancyGrid::SharedPtr m_currentMap;
    double m_mapResolution;
    double m_mapOriginX;
    double m_mapOriginY;
    int m_mapWidth;
    int m_mapHeight;
    
    // Robot pose
    double m_robotX;
    double m_robotY;
    double m_robotTheta;
    
    // Status
    bool m_isConnected;
    QString m_mapInfo;
    
    // Timers
    QTimer *m_spinTimer;
    QTimer *m_poseUpdateTimer;
    QTimer *m_velocityPublishTimer;
    
    // Image provider
    MapImageProvider* m_imageProvider;
    int m_imageCounter;
    
    // Current velocity state for continuous publishing
    double m_currentLinearVelocity;
    double m_currentAngularVelocity;
    bool m_isMoving;
};

#endif // ROS2MAPBRIDGE_H
