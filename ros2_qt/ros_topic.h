// ros_topic.h
#ifndef ROS_TOPIC_HANDLER_H
#define ROS_TOPIC_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <turtlesim/msg/pose.hpp>

#include <QMap>
#include <QTreeWidgetItem>
#include <QString>
#include <QObject>

class RosTopicHandler : public QObject  {
    Q_OBJECT
public:
    explicit RosTopicHandler(rclcpp::Node::SharedPtr node, QObject *parent = nullptr);
    ~RosTopicHandler();

    void subscribeTopic(const std::string &topic_name, const std::string &type_name);
    void publishTwist(const geometry_msgs::msg::Twist &twist_msg, const std::string &topic_name = "/turtle1/cmd_vel");

    void updateTreeItemForMessage(const QString &topic_name, QTreeWidgetItem *rootItem);

    void calculateFrequency(const QString &topic_name, const rclcpp::Time &timestamp, QTreeWidgetItem *item);

    QMap<QString, QTreeWidgetItem *> topic_data_container_items;

    // Current Twist command values (used by Qt slot to update UI)
    double current_linear_x_ = 0.0, current_linear_y_ = 0.0, current_linear_z_ = 0.0;
    double current_angular_x_ = 0.0, current_angular_y_ = 0.0, current_angular_z_ = 0.0;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    QMap<QString, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    QMap<QString, QVector<rclcpp::Time>> timestamp_histories_;

    void handlePoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const QString &topic_name);
    void handleTwist(const geometry_msgs::msg::Twist::SharedPtr msg, const QString &topic_name);
    void handlePointStamped(const geometry_msgs::msg::PointStamped::SharedPtr msg, const QString &topic_name);
    void handleTurtlePose(const turtlesim::msg::Pose::SharedPtr msg, const QString &topic_name);

    QTreeWidgetItem *findChildByText(QTreeWidgetItem *parent, const QString &text);
};

#endif // ROS_TOPIC_HANDLER_H
