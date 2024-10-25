/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Precision landing
 * @file precision_landing.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
// #include <px4_msgs/msg/landing_target_pose.hpp>
// #include <px4_msgs/msg/vehicle_odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class PrecisionLanding : public rclcpp::Node
{
public:
	PrecisionLanding() : Node("precision_landing")
	{
		
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		vehicle_gps_subscriber_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
		[this](const VehicleLocalPosition::SharedPtr msg) {
			loc_pos = msg;
			});


		offboard_setpoint_counter_ = 0;

		tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 50) {
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				// Arm the vehicle
				this->arm();
			}

			// Listen for tf
			listen_destination_position();
			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();
			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 51) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(20ms, timer_callback);

	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	geometry_msgs::msg::TransformStamped t0, t1;
	VehicleLocalPosition::SharedPtr loc_pos;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_gps_subscriber_;
	
	bool t0_detected;
	bool t1_detected;
	double yaw_t0;
	double yaw_t1; 

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;

	void listen_destination_position();
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void PrecisionLanding::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void PrecisionLanding::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void PrecisionLanding::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false; 
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Listen for destination position
 */
void PrecisionLanding::listen_destination_position()
{
	// VehicleOdometry msg{};

    try{
		this->t0 = this->tf_buffer_->lookupTransform("spiri_vision_0/camera_spiri/base_link/imager_down", "tag0", tf2::TimePointZero);
		this->t0_detected = true;
    }
    catch (tf2::TransformException &ex) {
		this->t0_detected = false;
    }

	try{
		this->t1 = this->tf_buffer_->lookupTransform("spiri_vision_0/camera_spiri/base_link/imager_down", "tag1", tf2::TimePointZero);
		this->t1_detected = true;
    }
    catch (tf2::TransformException &ex) {
		this->t1_detected = false;
    }
}

void PrecisionLanding::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	static int tampon = 0;
	double roll_hat, pitch_hat, yaw_hat;
	tf2::Quaternion q0;
	tf2::convert(t0.transform.rotation, q0);
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	// if (abs(loc_pos->x - 0) < 3 && abs(loc_pos->y - 0) < 3 && abs(t.transform.translation.x)>0 && abs(t.transform.translation.z)>=1){
	if (t0_detected==true && tampon>=250 && yaw_hat<0.1){ 
		
		std::cout << "Boucle 3 : " << tampon  << std::endl;

		msg.position[0] = std::numeric_limits<float>::quiet_NaN();
		msg.position[1] = std::numeric_limits<float>::quiet_NaN();
		msg.position[2] = std::numeric_limits<float>::quiet_NaN();
		
		msg.velocity[0] = -0.8*(cos(yaw_hat)*t0.transform.translation.x + sin(yaw_hat)*t0.transform.translation.y - 0.36);//-t.transform.translation.x; //OK
		// msg.velocity[1] = -0.8*(cos(yaw_hat)*t0.transform.translation.y - sin(yaw_hat)*t0.transform.translation.x + 0.3);
		msg.velocity[1] = -0.8*(cos(yaw_hat)*t0.transform.translation.y - sin(yaw_hat)*t0.transform.translation.x);
		msg.velocity[2] = 0.12;//*t0.transform.translation.z;//0.08*t.transform.translation.z; //MENE A DESCENTE -> OK
		msg.yaw = std::numeric_limits<float>::quiet_NaN();
		msg.yawspeed = yaw_hat;

		if (t1_detected==true && abs(t1.transform.translation.z)<0.8){ 
			std::cout << "Boucle 4 : " << tampon  << std::endl;
			tf2::convert(t1.transform.rotation, q0);
			msg.position[0] = std::numeric_limits<float>::quiet_NaN();
			msg.position[1] = std::numeric_limits<float>::quiet_NaN();
			msg.position[2] = std::numeric_limits<float>::quiet_NaN();

			// msg.velocity[0] = -0.3*t1.transform.translation.x;
			// msg.velocity[1] = -0.3*t1.transform.translation.y;

			msg.velocity[0] = -1*(cos(yaw_hat)*t1.transform.translation.x + sin(yaw_hat)*t1.transform.translation.y);//-t.transform.translation.x; //OK
			// msg.velocity[1] = -0.5*(cos(yaw_hat)*t1.transform.translation.y - sin(yaw_hat)*t1.transform.translation.x + 0.1); //OK
			msg.velocity[1] = -1*(cos(yaw_hat)*t1.transform.translation.y - sin(yaw_hat)*t1.transform.translation.x + 0.11);
			msg.velocity[2] = 0.12;

			msg.yaw = std::numeric_limits<float>::quiet_NaN();
			msg.yawspeed = yaw_hat; // ATTENTION CHAGER EN YAW_HAT1 !!!
			// if (abs(t1.transform.translation.z)<0.15 ){
			// 	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
			// }
			if (abs(t1.transform.translation.z)<0.25){

				std::cout << "Boucle 4 : " << tampon  << std::endl;
				msg.velocity[2] = 0.0;

				if (abs(t1.transform.translation.x)<0.05 && abs(t1.transform.translation.y + 0.11)<0.05){
					publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
				}
			}
		}
		
	}
	else if(t0_detected==true && abs(t0.transform.translation.x)<1 && abs(t0.transform.translation.y)<1 && abs(yaw_hat)<0.05){
		tampon++;
		msg.position[0] = 0;
		msg.position[1] = 0;
		msg.position[2] = - 2;
		msg.yaw = 1.57; // [-PI:PI]
		std::cout << "Boucle 2  : " << tampon  << std::endl;
	}
	else{
		tampon = 0;
		msg.position[0] = 0;
		msg.position[1] = 0;
		msg.position[2] = - 2;
		msg.yaw = 1.57; // [-PI:PI] connu car on suppose qu'on décolle de la plateforme
		std::cout << "Boucle 1 : " << tampon  << std::endl;
	}

	trajectory_setpoint_publisher_->publish(msg);

	tf2::Matrix3x3(q0).getRPY(roll_hat, pitch_hat, yaw_hat);

	// std::cout << "\n\n\n\n\n\n";
	std::cout << "RECEIVED CAMERA SENSOR DATA"   << std::endl;
	std::cout << "============================="   << std::endl;
	// // std::cout << "x: " << t.transform.translation.x  << std::endl;
	// // std::cout << "y: " << t.transform.translation.y  << std::endl;
	// // std::cout << "z: " << t.transform.translation.z  << std::endl;
	// std::cout << "roll: " << roll_hat  << std::endl;
	// std::cout << "pitch: " << pitch_hat << std::endl;
	// std::cout << "yaw: " << yaw_hat  << std::endl;

}



/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void PrecisionLanding::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting precision landing node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PrecisionLanding>());

	rclcpp::shutdown();
	return 0;
}
