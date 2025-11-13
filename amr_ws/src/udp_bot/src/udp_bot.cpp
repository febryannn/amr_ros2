#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <udp_bot/srv/KirimKecepatanUdp.hpp>
#include <udp_bot/srv/KirimOffsetUdp.hpp>
#include <udp_bot/msg/TerimaUdp.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <iostream>
#include <cstring>
#include <memory>
#include <chrono>

#include "udp_bot/UdpSocket.h"
#ifdef _WIN32
    #include "getopt.h"
#else
    #include <unistd.h>
#endif

#define TO_DEG 57.295779513
#define TO_RAD 0.01745329252

// Untuk ip tujuan kirim data dan portnya ETH
const char *addr = "169.254.1.15";
uint16_t port = 4444;
// Untuk bind dan local port ETH
const char *listenAddr = "169.254.1.16"; 
uint16_t localPort = 8888;

using namespace std::chrono_literals;

class UnicastApp {
public:
    rclcpp::Publisher<udp_bot::msg::TerimaUdp>::SharedPtr terima_udp_pub;
    rclcpp::Subscription<udp_bot::msg::KirimKecepatanUdp>::SharedPtr kirim_kecepatan_udp_sub;
    rclcpp::Subscription<udp_bot::msg::KirimOffsetUdp>::SharedPtr kirim_offset_udp_sub;
    udp_bot::msg::TerimaUdp msg;

    rclcpp::TimerBase::SharedPtr udp_send_timer;
    
    char stm_kirim[64] = {'t','e','l'};
    char stm_terima[70];

    // Terima udp
    float posisi_x_buffer, posisi_y_buffer, sudut_w_buffer;
    float kecepatan_robotx, kecepatan_roboty, kecepatan_robotw;
    int cnt_send_kecepatan;
    float sudut_servo_act;
    uint8_t tombol;
    float vx_global, vy_global, vw_global;
    
    // Kirim udp
    float kecepatan_x, kecepatan_y, kecepatan_sudut;
    float posisi_x_offset, posisi_y_offset, sudut_w_offset;
    float sudut_servo;

    // UDP Multicast
    UnicastApp(rclcpp::Node::SharedPtr node, const char *remoteAddr, const char *listenAddr, 
               uint16_t localPort, uint16_t port);

    virtual ~UnicastApp() = default;

    void onReceiveData(const char *data, size_t size);

    void sendMsg(const char *data, size_t len);

    void udp_kecepatan_send_callback(const udp_bot::msg::KirimKecepatanUdp::SharedPtr msg);

    void udp_offset_send_callback(const udp_bot::msg::KirimOffsetUdp::SharedPtr msg);

    void send_udp_data_callback();

private:
    rclcpp::Node::SharedPtr node_;
    sockets::SocketOpt m_socketOpts;
    sockets::UdpSocket<UnicastApp> m_unicast;
};

UnicastApp::UnicastApp(rclcpp::Node::SharedPtr node, const char *remoteAddr, 
                       const char *listenAddr, uint16_t localPort, uint16_t port) 
    : node_(node),
      m_socketOpts({ sockets::TX_BUFFER_SIZE, sockets::RX_BUFFER_SIZE, listenAddr}), 
      m_unicast(*this, &m_socketOpts) {
    
    cnt_send_kecepatan = 0;
    kecepatan_x = 0;
    kecepatan_y = 0;
    kecepatan_sudut = 0;
    sudut_servo = 0;
    posisi_x_offset = 0;
    posisi_y_offset = 0;
    sudut_w_offset = 0;

    terima_udp_pub = node_->create_publisher<udp_bot::msg::TerimaUdp>("data_terima_udp", 10);
    
    kirim_kecepatan_udp_sub = node_->create_subscription<udp_bot::msg::KirimKecepatanUdp>(
        "kecepatan_kirim_udp", 10, 
        std::bind(&UnicastApp::udp_kecepatan_send_callback, this, std::placeholders::_1));
    
    kirim_offset_udp_sub = node_->create_subscription<udp_bot::msg::KirimOffsetUdp>(
        "offset_kirim_udp", 10, 
        std::bind(&UnicastApp::udp_offset_send_callback, this, std::placeholders::_1));
    
    sockets::SocketRet ret = m_unicast.startUnicast(remoteAddr, localPort, port);
    if (ret.m_success) {
        RCLCPP_INFO(node_->get_logger(), "Listening on UDP %s:%d sending to %s:%d", 
                    listenAddr, localPort, remoteAddr, port);
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Error: %s", ret.m_msg.c_str());
        exit(1);
    }

    udp_send_timer = node_->create_wall_timer(
        10ms, std::bind(&UnicastApp::send_udp_data_callback, this));
}

void UnicastApp::sendMsg(const char *data, size_t len) {
    auto ret = m_unicast.sendMsg(data, len);
    if (!ret.m_success) {
        RCLCPP_ERROR(node_->get_logger(), "Send Error: %s", ret.m_msg.c_str());
    }
}

void UnicastApp::onReceiveData(const char *data, size_t size) {
    for(size_t i = 0; i < size; i++){
        stm_terima[i] = data[i];
    }

    memcpy(&posisi_x_buffer, stm_terima + 3, 4);
    memcpy(&posisi_y_buffer, stm_terima + 7, 4);
    memcpy(&sudut_w_buffer, stm_terima + 11, 4);
    memcpy(&kecepatan_robotx, stm_terima + 15, 4);
    memcpy(&kecepatan_roboty, stm_terima + 19, 4);
    memcpy(&kecepatan_robotw, stm_terima + 23, 4);
    memcpy(&sudut_servo_act, stm_terima + 27, 4);
    memcpy(&tombol, stm_terima + 31, 1);
    memcpy(&vx_global, stm_terima + 32, 4);
    memcpy(&vy_global, stm_terima + 36, 4);
    memcpy(&vw_global, stm_terima + 40, 4);

    msg.posisi_x_buffer = posisi_x_buffer;
    msg.posisi_y_buffer = posisi_y_buffer;
    msg.sudut_w_buffer = sudut_w_buffer;
    msg.kecepatan_robotx = kecepatan_robotx;
    msg.kecepatan_roboty = kecepatan_roboty;
    msg.kecepatan_robotw = kecepatan_robotw;
    msg.sudut_servo_act = sudut_servo_act;
    msg.tombol = tombol;
    msg.vx_global = vx_global;
    msg.vy_global = vy_global;
    msg.vw_global = vw_global;
    terima_udp_pub->publish(msg);
}

void UnicastApp::udp_kecepatan_send_callback(const udp_bot::msg::KirimKecepatanUdp::SharedPtr msg)
{
    kecepatan_x     = msg->kecepatan_x;
    kecepatan_y     = msg->kecepatan_y;
    kecepatan_sudut = msg->kecepatan_sudut;
    sudut_servo     = msg->sudut_servo;
    
    // For safety
    cnt_send_kecepatan = 0;
}

void UnicastApp::udp_offset_send_callback(const udp_bot::msg::KirimOffsetUdp::SharedPtr msg)
{
    posisi_x_offset = msg->posisi_x_offset; 
    posisi_y_offset = msg->posisi_y_offset;
    sudut_w_offset  = msg->sudut_w_offset;
}

void UnicastApp::send_udp_data_callback()
{   
    if(cnt_send_kecepatan >= 100) // Jika lebih dari 1s tidak terima kecepatan
    {
        kecepatan_x     = 0;
        kecepatan_y     = 0;
        kecepatan_sudut = 0;
    }
    if(cnt_send_kecepatan >= 100) 
        cnt_send_kecepatan = 100;
    else 
        cnt_send_kecepatan++;

    memcpy(stm_kirim + 3, &kecepatan_x, 4);
    memcpy(stm_kirim + 7, &kecepatan_y, 4);
    memcpy(stm_kirim + 11, &kecepatan_sudut, 4);
    memcpy(stm_kirim + 15, &posisi_x_offset, 4);
    memcpy(stm_kirim + 19, &posisi_y_offset, 4);
    memcpy(stm_kirim + 23, &sudut_w_offset, 4);
    memcpy(stm_kirim + 27, &sudut_servo, 4);

    UnicastApp::sendMsg(stm_kirim, sizeof(stm_kirim));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("udp_bot");

    auto *app = new UnicastApp(node, addr, listenAddr, localPort, port);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

