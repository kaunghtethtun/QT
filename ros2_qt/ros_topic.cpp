// ros_topic.cpp
#include "ros_topic.h"
#include <QDebug>


RosTopicHandler::RosTopicHandler(rclcpp::Node::SharedPtr node, QObject *parent)
    : QObject(parent), node_(node)
{
    rclcpp_thread_ = std::thread([this]() {
    rclcpp::spin(node_);
});

}
RosTopicHandler::~RosTopicHandler() {
    subscriptions_.clear();
    topic_data_container_items.clear();
    if (rclcpp_thread_.joinable()) {
    node_->get_node_base_interface()->get_context()->shutdown("QT GUI shutdown");
    rclcpp_thread_.join();
}

}

void RosTopicHandler::handlePoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const QString &topic_name)
{
    auto item = topic_data_container_items.value(topic_name);
    if (!item) return;

    QString value_str = QString("position: [x: %1, y: %2, z: %3], orientation: [x: %4, y: %5, z: %6, w: %7]")
                            .arg(msg->pose.position.x, 0, 'f', 3)
                            .arg(msg->pose.position.y, 0, 'f', 3)
                            .arg(msg->pose.position.z, 0, 'f', 3)
                            .arg(msg->pose.orientation.x, 0, 'f', 3)
                            .arg(msg->pose.orientation.y, 0, 'f', 3)
                            .arg(msg->pose.orientation.z, 0, 'f', 3)
                            .arg(msg->pose.orientation.w, 0, 'f', 3);

    item->setText(2, value_str);
}

void RosTopicHandler::handlePointStamped(const geometry_msgs::msg::PointStamped::SharedPtr msg, const QString &topic_name)
{
    auto item = topic_data_container_items.value(topic_name);
    if (!item) return;

    QString value_str = QString("point: [x: %1, y: %2, z: %3]")
                            .arg(msg->point.x, 0, 'f', 3)
                            .arg(msg->point.y, 0, 'f', 3)
                            .arg(msg->point.z, 0, 'f', 3);

    item->setText(2, value_str);
}

void RosTopicHandler::handleTurtlePose(
    turtlesim::msg::Pose::SharedPtr msg,
    const QString &topic_name)
{
     auto item = topic_data_container_items.value(topic_name);
    if (!item) return;

    QString value_str = QString("position: [x: %1, y: %2, z: %3], orientation: [x: %4, y: %5, z: %6, w: %7]")
                            .arg(msg->pose.position.x, 0, 'f', 3)
                            .arg(msg->pose.position.y, 0, 'f', 3)
                            .arg(msg->pose.position.z, 0, 'f', 3)
                            .arg(msg->pose.orientation.x, 0, 'f', 3)
                            .arg(msg->pose.orientation.y, 0, 'f', 3)
                            .arg(msg->pose.orientation.z, 0, 'f', 3)
                            .arg(msg->pose.orientation.w, 0, 'f', 3);

    QTimer::singleShot(0, [item, value_str]() {
        item->setText(2, value_str);  // ✅ Safe
    });
    //qDebug() << "[ROS CALLBACK] topic:" << topic_name << ", value:" << value_str;

}


void RosTopicHandler::subscribeTopic(const std::string &topic_name_std, const std::string &type_name) {
    QString topic_name = QString::fromStdString(topic_name_std);

    if (type_name == "geometry_msgs/msg/PoseStamped") {
        auto sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_name_std, rclcpp::QoS(10),
            [this, topic_name](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                handlePoseStamped(msg, topic_name);
            });
        subscriptions_[topic_name] = sub;
    } else if (type_name == "geometry_msgs/msg/Twist") {
        auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
            topic_name_std, rclcpp::QoS(10),
            [this, topic_name](geometry_msgs::msg::Twist::SharedPtr msg) {
                handleTwist(msg, topic_name);
            });
        subscriptions_[topic_name] = sub;
    } else if (type_name == "geometry_msgs/msg/PointStamped") {
        auto sub = node_->create_subscription<geometry_msgs::msg::PointStamped>(
            topic_name_std, rclcpp::QoS(10),
            [this, topic_name](geometry_msgs::msg::PointStamped::SharedPtr msg) {
                handlePointStamped(msg, topic_name);
            });
        subscriptions_[topic_name] = sub;
    } else if (type_name == "turtlesim/msg/Pose") {
        auto sub = node_->create_subscription<turtlesim::msg::Pose>(
            topic_name_std, rclcpp::QoS(10),
            [this, topic_name](turtlesim::msg::Pose::SharedPtr msg) {
                handleTurtlePose(msg, topic_name);
            });
        subscriptions_[topic_name] = sub;
    }

}

void RosTopicHandler::publishTwist(const geometry_msgs::msg::Twist &twist_msg, const std::string &topic_name) {
    if (!cmd_vel_pub_) {
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic_name, 10);
    }
    cmd_vel_pub_->publish(twist_msg);
}

// Implement the handle* functions as needed (like you already do in `updateTwistData`, `updatePoseStampedData`, etc.)

// Example
void RosTopicHandler::handleTwist(const geometry_msgs::msg::Twist::SharedPtr msg, const QString &topic_name) {
    // Locate data container (can set value directly in Qt UI)
    QTreeWidgetItem *dataContainerItem = topic_data_container_items.value(topic_name);
    if (!dataContainerItem) return;

    QTreeWidgetItem *linear = findChildByText(dataContainerItem, "Linear");
    QTreeWidgetItem *angular = findChildByText(dataContainerItem, "Angular");

    if (!linear || !angular) return;

    findChildByText(linear, "x:")->setText(2, QString::number(msg->linear.x, 'f', 3));
    findChildByText(linear, "y:")->setText(2, QString::number(msg->linear.y, 'f', 3));
    findChildByText(linear, "z:")->setText(2, QString::number(msg->linear.z, 'f', 3));
    findChildByText(angular, "x:")->setText(2, QString::number(msg->angular.x, 'f', 3));
    findChildByText(angular, "y:")->setText(2, QString::number(msg->angular.y, 'f', 3));
    findChildByText(angular, "z:")->setText(2, QString::number(msg->angular.z, 'f', 3));

    calculateFrequency(topic_name, node_->now(), topic_data_container_items.value(topic_name));
}

QTreeWidgetItem *RosTopicHandler::findChildByText(QTreeWidgetItem *parent, const QString &text) {
    for (int i = 0; i < parent->childCount(); ++i) {
        if (parent->child(i)->text(1) == text)
            return parent->child(i);
    }
    return nullptr;
}

void RosTopicHandler::calculateFrequency(const QString &topic_name, const rclcpp::Time &timestamp, QTreeWidgetItem *item) {
    QVector<rclcpp::Time> &history = timestamp_histories_[topic_name];
    history.push_back(timestamp);
    if (history.size() > 100) history.removeFirst();

    if (history.size() >= 2) {
        double duration = (history.last() - history.first()).seconds();
        double freq = (history.size() - 1) / duration;
        item->setText(3, QString::number(freq, 'f', 2) + " Hz");
    }
}
