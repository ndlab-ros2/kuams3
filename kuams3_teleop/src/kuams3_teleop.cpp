#include <rclcpp/rclcpp.hpp>  //ROS2のC++クライアントライブラリ(rclcpp)を提供する
#include <geometry_msgs/msg/twist.hpp>  //ロボットの速度情報を表現するための「Twist」メッセージ型を使用するためのヘッダファイルの読み込み
#include <sensor_msgs/msg/joy.hpp>  //ジョイスティックの入力データを表す「Joy」メッセージ型を使用するためのヘッダファイルの読み込み
#include <mutex>  //複数のスレッドが同時に共有データを操作しないように制御する仕組みのmutex機能を読み込む

//rclcpp::Nodeを継承して、Kuams3Teleopを定義する
class Kuams3Teleop : public rclcpp::Node
{
public:
    //ノードの名前を「kuams3_teleop」と設定
    Kuams3Teleop() : Node("kuams3_teleop")
    {
        //パラメータの説明
        this->declare_parameter<int>("axis_linear", 1);  //並進運動に関するパラメータ
        this->declare_parameter<int>("axis_angular", 0);  //回転運動に関するパラメータ
        this->declare_parameter<int>("axis_deadman", 4);  //操作判定ボタンに関するパラメータ
        this->declare_parameter<double>("scale_linear", 0.3);  //並進運動の最大速度
        this->declare_parameter<double>("scale_angular", 0.9);  //回転運動の最大速度

        //先程説明したパラメータの取得
        this->get_parameter("axis_linear", linear_axis_);
        this->get_parameter("axis_angular", angular_axis_);
        this->get_parameter("axis_deadman", deadman_axis_);
        this->get_parameter("scale_linear", linear_scale_);
        this->get_parameter("scale_angular", angular_scale_);

        //フラグの初期化
        deadman_pressed_ = false;  //デッドマンボタン(操作継続を保証するためのボタン)が押されているかを追跡する
        zero_twist_published_ = false;  //停止コマンドが送信済みであるかを追跡する

        // Create publisher and subscriber
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);  //cmd_velというトピックを送信するパブリッシャーを作成する
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Kuams3Teleop::joyCallback, this, std::placeholders::_1)); //joyトピックを受け取るサブスクライバーの作成　joyCallback:コールバック関数

        // Create a timer to publish at regular intervals
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Kuams3Teleop::publish, this));  //100ミリセカンドごとにpulish関数を呼ぶタイマーの作成
    }

private:
    //ジョイスティック入力を処理するコールバック関数
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)  //
    {
        auto vel = geometry_msgs::msg::Twist();
        vel.angular.z = angular_scale_ * joy->axes[angular_axis_];
        vel.linear.x = linear_scale_ * joy->axes[linear_axis_];
        last_published_ = vel;
        deadman_pressed_ = joy->buttons[deadman_axis_];  //デッドマンボタンの状態を取得して、デッドマンのフラグを更新する
    }

    void publish()
    {
        std::lock_guard<std::mutex> lock(publish_mutex_);

        if (deadman_pressed_)
        {
            vel_pub_->publish(last_published_);
            zero_twist_published_ = false;
        }
        else if (!deadman_pressed_ && !zero_twist_published_)
        {
            vel_pub_->publish(geometry_msgs::msg::Twist());
            zero_twist_published_ = true;
        }
    }

    // Node parameters
    int linear_axis_, angular_axis_, deadman_axis_;
    double linear_scale_, angular_scale_;

    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Timers and mutex
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex publish_mutex_;

    // State variables
    geometry_msgs::msg::Twist last_published_;
    bool deadman_pressed_;
    bool zero_twist_published_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Kuams3Teleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
