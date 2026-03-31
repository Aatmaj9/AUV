#include "dvl_a50/dvl-sensor.hpp"

namespace dvl_sensor {

DVL_A50::DVL_A50() : Node("dvl_a50_node")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
        qos_profile);

    timer_receive = this->create_wall_timer(
        std::chrono::milliseconds(40),
        std::bind(&DVL_A50::handle_receive, this));

    dvl_pub_report = this->create_publisher<dvl_msgs::msg::DVL>("dvl/data", qos);
    dvl_pub_pos = this->create_publisher<dvl_msgs::msg::DVLDR>("dvl/position", qos);
    dvl_pub_config_status = this->create_publisher<dvl_msgs::msg::ConfigStatus>("dvl/config/status", qos);
    dvl_pub_command_response = this->create_publisher<dvl_msgs::msg::CommandResponse>("dvl/command/response", qos);

    dvl_sub_config_command = this->create_subscription<dvl_msgs::msg::ConfigCommand>(
        "dvl/config/command", qos,
        std::bind(&DVL_A50::command_subscriber, this, std::placeholders::_1));

    this->declare_parameter<std::string>("dvl_ip_address", "192.168.194.95");
    this->declare_parameter<std::string>("velocity_frame_id", "dvl_A50/velocity_link");
    this->declare_parameter<std::string>("position_frame_id", "dvl_A50/position_link");

    velocity_frame_id = this->get_parameter("velocity_frame_id").as_string();
    position_frame_id = this->get_parameter("position_frame_id").as_string();
    ip_address = this->get_parameter("dvl_ip_address").as_string();

    RCLCPP_INFO(get_logger(), "Connecting to DVL at: %s", ip_address.c_str());

    tcpSocket = new TCPSocket((char*)ip_address.c_str(), 16171);

    if (tcpSocket->Create() < 0)
        RCLCPP_ERROR(get_logger(), "Error creating socket");

    tcpSocket->SetRcvTimeout(400);

    std::string error;
    int error_code = 0;

    auto start_time = std::chrono::steady_clock::now();
    while (fault != 0) {
        fault = tcpSocket->Connect(5000, error, error_code);

        if (error_code == 114 || error_code == 103) {
            RCLCPP_WARN(get_logger(), "Waiting for DVL... (%d)", error_code);
            usleep(2000000);

            if (std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start_time).count() > 80.0) {
                fault = -10;
                break;
            }
        } else if (fault != 0) break;
    }

    if (fault == -10) {
        tcpSocket->Close();
        RCLCPP_ERROR(get_logger(), "DVL connection failed!");
    } else {
        RCLCPP_INFO(get_logger(), "DVL-A50 connected!");

        
        this->set_json_parameter("acoustic_enabled", "true");
    }
}

DVL_A50::~DVL_A50() {
    tcpSocket->Close();
    delete tcpSocket;
}

void DVL_A50::handle_receive() {
    if (fault != 0) return;

    while (true) {
        char tempBuffer[1];
        std::string str = "";
        int chars_read = 0;
        const int max_chars = 3000;
        bool found_newline = false;

        while (chars_read < max_chars) {
            int r = tcpSocket->Receive(tempBuffer);

            if (r > 0) {
                if (tempBuffer[0] == '\n') {
                    found_newline = true;
                    break;
                }
                str += tempBuffer[0];
                chars_read++;
            } else {
                return; // socket empty
            }
        }

        if (!found_newline || str.empty() || str[0] != '{')
            continue;

        try {
            json_data = json::parse(str);

            if (json_data.contains("altitude"))
                publish_vel_trans_report();
            else if (json_data.contains("pitch"))
                publish_dead_reckoning_report();
            else if (json_data.contains("response_to")) {
                if (json_data["response_to"] == "get_config")
                    publish_config_status();
                else
                    publish_command_response();
            }

        } catch (...) {
        
        }
    }
}


void DVL_A50::publish_vel_trans_report() {

    dvl_msgs::msg::DVL dvl;
    dvl.header.stamp = this->now();
    dvl.header.frame_id = velocity_frame_id;

    if (json_data.contains("time"))
        dvl.time = double(json_data["time"]);

    if (json_data.contains("vx")) dvl.velocity.x = double(json_data["vx"]);
    if (json_data.contains("vy")) dvl.velocity.y = double(json_data["vy"]);
    if (json_data.contains("vz")) dvl.velocity.z = double(json_data["vz"]);

    if (json_data.contains("altitude"))
        dvl.altitude = double(json_data["altitude"]);

    if (json_data.contains("velocity_valid"))
        dvl.velocity_valid = json_data["velocity_valid"];

    if (json_data.contains("status"))
        dvl.status = json_data["status"];


    if (json_data.contains("covariance") && json_data["covariance"].is_array()) {
        std::vector<double> cov;

        for (auto &row : json_data["covariance"]) {
            if (!row.is_array()) continue;
            for (auto &val : row) {
                if (val.is_number())
                    cov.push_back(val.get<double>());
            }
        }
        dvl.covariance = cov;
    }

 
    if (json_data.contains("transducers") &&
        json_data["transducers"].is_array() &&
        json_data["transducers"].size() >= 4) {

        for (int i = 0; i < 4; i++) {
            dvl_msgs::msg::DVLBeam beam;
            auto &t = json_data["transducers"][i];

            if (t.contains("id")) beam.id = t["id"];
            if (t.contains("velocity")) beam.velocity = double(t["velocity"]);
            if (t.contains("distance")) beam.distance = double(t["distance"]);
            if (t.contains("rssi")) beam.rssi = double(t["rssi"]);
            if (t.contains("nsd")) beam.nsd = double(t["nsd"]);
            if (t.contains("beam_valid")) beam.valid = t["beam_valid"];

            dvl.beams.push_back(beam);
        }
    }

    dvl_pub_report->publish(dvl);
}

void DVL_A50::publish_dead_reckoning_report() {

    dvl_msgs::msg::DVLDR msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = position_frame_id;

    if (json_data.contains("x")) msg.position.x = double(json_data["x"]);
    if (json_data.contains("y")) msg.position.y = double(json_data["y"]);
    if (json_data.contains("z")) msg.position.z = double(json_data["z"]);

    if (json_data.contains("roll")) msg.roll = double(json_data["roll"]);
    if (json_data.contains("pitch")) msg.pitch = double(json_data["pitch"]);
    if (json_data.contains("yaw")) msg.yaw = double(json_data["yaw"]);

    dvl_pub_pos->publish(msg);
}


void DVL_A50::publish_command_response() {
    dvl_msgs::msg::CommandResponse resp;

    if (json_data.contains("response_to"))
        resp.response_to = json_data["response_to"];

    if (json_data.contains("success"))
        resp.success = json_data["success"];

    dvl_pub_command_response->publish(resp);
}


void DVL_A50::publish_config_status() {
    dvl_msgs::msg::ConfigStatus status;

    if (json_data.contains("response_to"))
        status.response_to = json_data["response_to"];

    dvl_pub_config_status->publish(status);
}


void DVL_A50::set_json_parameter(const std::string name, const std::string value) {
    json msg;
    msg["command"] = "set_config";
    msg["parameters"][name] = value;

    std::string s = msg.dump();
    tcpSocket->Send((char*)s.c_str());
}


void DVL_A50::command_subscriber(
    const dvl_msgs::msg::ConfigCommand::SharedPtr msg) {

    json command;
    command["command"] = msg->command;

    if (msg->command == "set_config")
        command["parameters"][msg->parameter_name] = msg->parameter_value;

    std::string s = command.dump();
    tcpSocket->Send((char*)s.c_str());
}

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dvl_sensor::DVL_A50>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}