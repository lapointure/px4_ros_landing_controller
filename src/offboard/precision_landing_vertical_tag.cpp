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
 * @brief Precision landing using vertical AprilTags
 * @file precision_landing_vertical_tag.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class PrecisionLandingVerticalTag : public rclcpp::Node
{
public:
	PrecisionLandingVerticalTag() : Node("precision_landing_vertical_tag")
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
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

		auto timer_callback = [this]() -> void {
			// trajectory_setpoint has to be published for 1 second before switching to offboard mode
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
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
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);

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
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	geometry_msgs::msg::TransformStamped t0; 
	geometry_msgs::msg::TransformStamped t1; 
	geometry_msgs::msg::TransformStamped tf_b_c; //camera % base_link tf_b_c = tf_parent_child

	geometry_msgs::msg::TransformStamped tf_t0_p; //plateforme % tag tf_t0_p
	geometry_msgs::msg::TransformStamped tf_t1_p; //plateforme % tag tf_t1_p

	geometry_msgs::msg::TransformStamped tf_c_t0; //tag % camera tf_c_t0
	geometry_msgs::msg::TransformStamped tf_c_t1; //tag % camera tf_c_t1

	bool t0_detected;
	bool t1_detected;
	double yaw_t0;
	double yaw_t1; 

	VehicleLocalPosition::SharedPtr loc_pos;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_gps_subscriber_;
	


	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void listen_destination_position();
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void PrecisionLandingVerticalTag::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void PrecisionLandingVerticalTag::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void PrecisionLandingVerticalTag::publish_offboard_control_mode()
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
void PrecisionLandingVerticalTag::listen_destination_position()
{
	double roll, pitch;
	// static int tampon2 = 0;

	tf_b_c.header.stamp = this->get_clock()->now();
    tf_b_c.header.frame_id = "spiri_vision_0/base_link";
    tf_b_c.child_frame_id = "spiri_vision_0/camera_spiri/base_link/imager_front";
	tf_b_c.transform.translation.x = 0.2;
    tf_b_c.transform.translation.y = 0;
    tf_b_c.transform.translation.z = 0;
	tf2::Quaternion q_b_c;
	q_b_c.setRPY(-2.09, 0, -1.57); //caméra 30° vers le bas
    tf_b_c.transform.rotation.x = q_b_c.x();
    tf_b_c.transform.rotation.y = q_b_c.y();
    tf_b_c.transform.rotation.z = q_b_c.z();
    tf_b_c.transform.rotation.w = q_b_c.w();
	

	tf2::Quaternion q_p_t0;
    q_p_t0.setRPY(1.57, 0, -1.57); //tag vertical
	tf2::Transform tf_p_t0;
	// tf_p_t0.setOrigin(tf2::Vector3(0.5, 0.28, 0.15)); // 0.2 car on veut la hauteur par rapport à la surface de la platefrome en z (voir frames dans gazebo)
	tf_p_t0.setOrigin(tf2::Vector3(0.35, 0.28, 0.15));
	tf_p_t0.setRotation(tf2::Quaternion(q_p_t0));

	tf2::Quaternion q_p_t1;
    q_p_t1.setRPY(1.57, 0, -1.57); //tag vertical
	tf2::Transform tf_p_t1;
	// tf_p_t1.setOrigin(tf2::Vector3(0.5, 0, 0.06)); // 0.11 car on veut la hauteur par rapport à la surface de la platefrome en z (voir frames dans gazebo)
	tf_p_t1.setOrigin(tf2::Vector3(0.35, 0, 0.06));
	tf_p_t1.setRotation(tf2::Quaternion(q_p_t1));

	tf2::Transform tf_t0_p_unstamped = tf_p_t0.inverse();

	tf2::toMsg(tf_t0_p_unstamped, tf_t0_p.transform);

	tf2::Transform tf_t1_p_unstamped = tf_p_t1.inverse();

	tf2::toMsg(tf_t1_p_unstamped, tf_t1_p.transform);

	this->t0_detected = false;
	this->t1_detected = false;
	
	try{
		tf_c_t0 = this->tf_buffer_->lookupTransform("spiri_vision_0/camera_spiri/base_link/imager_front", "tag0", tf2::TimePointZero);
		// tf2::Quaternion q3;
		// tf2::convert(tf_c_t0.transform.rotation, q3);
		// tf2::Matrix3x3(q3).getRPY(roll, pitch, yaw);

		tf2::Transform tr1, tr2, tr3, t_unstamped;
		tf2::fromMsg(tf_b_c.transform, tr1);
		tf2::fromMsg(tf_t0_p.transform, tr2);
		tf2::fromMsg(tf_c_t0.transform, tr3);

		t_unstamped = tr1*tr3*tr2;

		t0.transform = tf2::toMsg(t_unstamped);

		tf2::Quaternion q;
		tf2::convert(t0.transform.rotation, q);
		tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_t0);

		this->t0_detected = true;

		// std::cout << "TAG 0";

	}
	catch(tf2::TransformException & ex){
		this->t0_detected = false;
		return;
	}

	try{
		tf_c_t1 = this->tf_buffer_->lookupTransform("spiri_vision_0/camera_spiri/base_link/imager_front", "tag1", tf2::TimePointZero);
		// tf2::Quaternion q3;
		// tf2::convert(tf_c_t1.transform.rotation, q3);
		// tf2::Matrix3x3(q3).getRPY(roll, pitch, yaw);

		tf2::Transform tr1, tr2, tr3, t_unstamped;
		tf2::fromMsg(tf_b_c.transform, tr1);
		tf2::fromMsg(tf_t1_p.transform, tr2);
		tf2::fromMsg(tf_c_t1.transform, tr3);

		t_unstamped = tr1*tr3*tr2;

		t1.transform = tf2::toMsg(t_unstamped);

		tf2::Quaternion q;
		tf2::convert(t1.transform.rotation, q);
		tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_t1);

		this->t1_detected = true;

		// std::cout << "TAG 1";

	}
	catch(tf2::TransformException &ex){
		this->t1_detected = false;
		return;
	}

	std::cout << "\n\n\n\n\n\n";
	std::cout << "RECEIVED FRONT CAMERA SENSOR DATA"   << std::endl;
	std::cout << "============================="   << std::endl;
	std::cout << "x: " << t1.transform.translation.x  << std::endl;
	std::cout << "y: " << t1.transform.translation.y  << std::endl;
	std::cout << "z: " << t1.transform.translation.z  << std::endl;
	std::cout << "roll: " << roll  << std::endl;
	std::cout << "pitch: " << pitch << std::endl;
	std::cout << "yaw_t0: " << yaw_t0  << std::endl;
	std::cout << "yaw_t1: " << yaw_t1  << std::endl;
	std::cout << "t0_detected: " << t0_detected  << std::endl;
	std::cout << "t1_detected: " << t1_detected  << std::endl;
    
}

void PrecisionLandingVerticalTag::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};

	static int tampon = 0;	


	if (t0_detected==true && tampon>=50){ 
		
		std::cout << "Boucle 3 : " << tampon  << std::endl;

		msg.position[0] = std::numeric_limits<float>::quiet_NaN();
		msg.position[1] = std::numeric_limits<float>::quiet_NaN();
		msg.position[2] = std::numeric_limits<float>::quiet_NaN();
		
		msg.velocity[0] = 0.5*t0.transform.translation.y/abs(t0.transform.translation.y)*std::min(abs(0.8*t0.transform.translation.y), 0.5); //OK
		msg.velocity[1] = 0.5*t0.transform.translation.x/abs(t0.transform.translation.x)*std::min(abs(0.8*t0.transform.translation.x), 0.5); //OK
		// msg.velocity[2] = 0.3;
		msg.velocity[2] = -t0.transform.translation.x - t0.transform.translation.z + 0.1;
		if (abs(t0.transform.translation.z)<0.2){
			msg.velocity[2] = 0;
		}
		
		msg.yaw = std::numeric_limits<float>::quiet_NaN();
		msg.yawspeed = -0.5*yaw_t0;


		if (t1_detected==true){
			// std::cout << "Boucle 4 : " << tampon  << std::endl;
			msg.position[0] = std::numeric_limits<float>::quiet_NaN();
			msg.position[1] = std::numeric_limits<float>::quiet_NaN();
			msg.position[2] = std::numeric_limits<float>::quiet_NaN();
			
			// msg.velocity[0] = 0.5*t1.transform.translation.y; //OK
			// msg.velocity[1] = 0.5*t1.transform.translation.x; //OK
			msg.velocity[0] = 0.5*t1.transform.translation.y/abs(t1.transform.translation.y)*std::min(abs(0.8*t1.transform.translation.y), 0.3); //OK
			msg.velocity[1] = 0.5*t1.transform.translation.x/abs(t1.transform.translation.x)*std::min(abs(0.8*t1.transform.translation.x), 0.3); //OK
			msg.velocity[2] = -t1.transform.translation.x - t1.transform.translation.z + 0.1;
			msg.yaw = std::numeric_limits<float>::quiet_NaN();
			msg.yawspeed = -0.5*yaw_t1;

			if (abs(t1.transform.translation.z)<0.15){
				msg.velocity[2] = 0;
			}
			
			if (abs(t1.transform.translation.x)<0.05 && abs(t1.transform.translation.x)<0.05 && abs(yaw_t1)<0.03){

				std::cout << "Boucle 4 : " << tampon  << std::endl;
				msg.velocity[2] = 0.0;

				if (abs(t1.transform.translation.z)<0.15){
					publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
				}
			}
		}
		}
		else if(t0_detected==true && abs(t0.transform.translation.x - 1)<0.5 && abs(t0.transform.translation.y)<0.5){   //HOVERING PHASE
		tampon++;
		msg.position[0] = std::numeric_limits<float>::quiet_NaN();
		msg.position[1] = std::numeric_limits<float>::quiet_NaN();
		msg.position[2] = std::numeric_limits<float>::quiet_NaN();
		
		msg.velocity[0] = 0.5*(0 - loc_pos->x); //OK
		msg.velocity[1] = 0.5*(-1 - loc_pos->y); //OK
		msg.velocity[2] = 0.3*(-0.9 - loc_pos->z);

		msg.yaw = 1.57; // [-PI:PI]
		std::cout << "Boucle 2  : " << tampon  << std::endl;
		std::cout << "x: " << loc_pos->x  << std::endl;
		std::cout << "y: " << loc_pos->y  << std::endl;
		std::cout << "z: " << loc_pos->z  << std::endl;
	}
	else{                   // PHASE D'APPROCHE
		tampon = 0;
		msg.position[0] = std::numeric_limits<float>::quiet_NaN();
		msg.position[1] = std::numeric_limits<float>::quiet_NaN();
		msg.position[2] = std::numeric_limits<float>::quiet_NaN();
		
		msg.velocity[0] = 0.5*(0 - loc_pos->x); //OK
		msg.velocity[1] = 0.5*(-1 - loc_pos->y); //OK
		msg.velocity[2] = 0.3*(-0.9 - loc_pos->z);

		msg.yaw = 1.57; // [-PI:PI] connu car on suppose qu'on décolle de la plateforme
		std::cout << "Boucle 1 : " << tampon  << std::endl;
		std::cout << "x: " << loc_pos->x  << std::endl;
		std::cout << "y: " << loc_pos->y  << std::endl;
		std::cout << "z: " << loc_pos->z  << std::endl;

		if (abs(loc_pos->x - 0)<0.5 && abs(loc_pos->y + 1)<0.5 && abs(loc_pos->z + 1)<0.5 && t0_detected==false){

			std::cout << "Boucle 5 : " << tampon  << std::endl;
			msg.position[0] = std::numeric_limits<float>::quiet_NaN();
			msg.position[1] = std::numeric_limits<float>::quiet_NaN();
			msg.position[2] = std::numeric_limits<float>::quiet_NaN();
			msg.yaw = std::numeric_limits<float>::quiet_NaN();
			
			msg.velocity[0] = 0; //OK
			msg.velocity[1] = 0; //OK
			msg.velocity[2] = 0;
			msg.yawspeed = 0.2;
		}
		
	}

	if (t0_detected==false && t1_detected==false){
		tampon = 0;
		msg.position[0] = 0;
		msg.position[1] = -1;
		msg.position[2] = - 1.3;
		msg.yaw = 1.57;
	}


	trajectory_setpoint_publisher_->publish(msg);

}



/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void PrecisionLandingVerticalTag::publish_vehicle_command(uint16_t command, float param1, float param2)
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
	std::cout << "Starting precision landing with verical tag node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PrecisionLandingVerticalTag>());

	rclcpp::shutdown();
	return 0;
}
