MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Start at Home Page
    ui->stackedWidget->setCurrentIndex(0);

    // Create ROS Node
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("qt_gui_node");

    connect(ui->pushButton_GoToList, &QPushButton::clicked, this, &MainWindow::onGoToListClicked);
    connect(ui->pushButton_Subscribe, &QPushButton::clicked, this, &MainWindow::onSubscribeClicked);
    connect(ui->pushButton_Publish, &QPushButton::clicked, this, &MainWindow::onPublishClicked);
    connect(ui->pushButton_Back, &QPushButton::clicked, this, &MainWindow::onBackToListClicked);
    connect(ui->pushButton_SendMessage, &QPushButton::clicked, this, &MainWindow::onPublishMessageClicked);
}

MainWindow::~MainWindow()
{
    rclcpp::shutdown();
}

// Page navigation
void MainWindow::onGoToListClicked()
{
    ui->stackedWidget->setCurrentIndex(1);  // Topic List Page
    // TODO: Fill topic list from ROS
}

void MainWindow::onSubscribeClicked()
{
    selectedTopic_ = ui->listWidget_Topics->currentItem()->text();
    
    subscriber_ = node_->create_subscription<std_msgs::msg::String>(
        selectedTopic_.toStdString(),
        10,
        [this](std_msgs::msg::String::UniquePtr msg)
        {
            QString message = QString::fromStdString(msg->data);
            ui->label_SubscribeMessage->setText(message);  // Assume you have a label in Page 3
        }
    );

    ui->stackedWidget->setCurrentIndex(2);  // Subscribe Page
}

void MainWindow::onPublishClicked()
{
    selectedTopic_ = ui->listWidget_Topics->currentItem()->text();

    publisher_ = node_->create_publisher<std_msgs::msg::String>(selectedTopic_.toStdString(), 10);

    ui->stackedWidget->setCurrentIndex(3);  // Publish Page
}

void MainWindow::onBackToListClicked()
{
    ui->stackedWidget->setCurrentIndex(1); // Go back to Topic List
}

void MainWindow::onPublishMessageClicked()
{
    if (publisher_)
    {
        std_msgs::msg::String msg;
        msg.data = ui->lineEdit_InputMessage->text().toStdString();
        publisher_->publish(msg);
    }
}
