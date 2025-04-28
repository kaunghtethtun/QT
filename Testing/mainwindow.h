#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>  // example message type

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onGoToListClicked();
    void onSubscribeClicked();
    void onPublishClicked();
    void onBackToListClicked();
    void onPublishMessageClicked();  // For sending message

private:
    Ui::MainWindow *ui;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    QString selectedTopic_;
};
