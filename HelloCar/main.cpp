
#pragma once
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/VectorMath.hpp"
#include <future>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <cstdlib>
#include "ImageProcessor.h"
#include "TcpStatechartClient.h"
#include <algorithm>

using namespace msr::airlib;

typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
typedef common_utils::FileSystem FileSystem;
typedef CarApiBase::CarState CarState;
typedef Eigen::Vector2f Vector2f;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector4f Vector4f;
typedef TcpStatechartClient TcpClient;
typedef CarApiBase::CarControls CarControls;

const int RPC_TIMEOUT = 6000000;
long long overallTime;

volatile float current_distance[] = { 0.0f , 0.0f,0.0f, 0.0f };
volatile float target_angle[] = { 0.0f , 0.0f,0.0f, 0.0f };
volatile float steering_sensitivity[] = { 1.0f , 1.0f, 1.0f, 1.0f};
volatile float brake_track[] = { 0.0f , 0.0f , 0.0f, 0.0f};

enum Mode { vector_calculation, image_processing };
int current_mode = Mode::vector_calculation;

const float MIN_STOP_DISTANCE = 7.0f;    //minimal distance to keep between vehicles



void processImages(CarRpcLibClient& client, int segmentationId, int sessionID) {
	ImageProcessor *processor = new ImageProcessor(segmentationId);
	while (client.getConnectionState() == RpcLibClientBase::ConnectionState::Connected) {
		auto future = std::async(std::launch::async, [&] {
			target_angle[sessionID-1] = processor->calculateSteeringAngle(client) * 0.1f;
			current_distance[sessionID-1] = processor->calculateDistance(client);
		});
	}
	delete processor;
	processor = NULL;
}

/* 3D sim vector math -
Computation of target angle between following and leading car.
Computation of distance based on 3d objects pose
*/
void processVectorCalculations(CarState& lead_state, CarState& follower_state, int sessionID) {
	Pose lead_pose = lead_state.kinematics_true.pose;
	Pose follower_pose = follower_state.kinematics_true.pose;

	Quaternionr quat = follower_pose.orientation;
	Vector3f rotated_x_axis = VectorMath::rotateVector(Vector3f::UnitX(), quat, false);
	Vector3f rotated_y_axis = VectorMath::rotateVector(Vector3f::UnitY(), quat, false);
	Vector3f rotated_z_axis = VectorMath::rotateVector(Vector3f::UnitZ(), quat, false);
	Eigen::Matrix4f m;
	m << rotated_x_axis.x(), rotated_y_axis.x(), rotated_z_axis.x(), follower_pose.position.x(),
		rotated_x_axis.y(), rotated_y_axis.y(), rotated_z_axis.y(), follower_pose.position.y(),
		rotated_x_axis.z(), rotated_y_axis.z(), rotated_z_axis.z(), follower_pose.position.z(),
		0.0f, 0.0f, 0.0f, 1.0f;

	Eigen::Matrix4f m_inverted = m.inverse();
	Eigen::Vector4f new_lead_pos = m_inverted * Eigen::Vector4f(lead_pose.position.x(), lead_pose.position.y(), lead_pose.position.z(), 1.0f);
	float angle = static_cast<float>(atan2f(new_lead_pos.y(), new_lead_pos.x())* (180.0f / std::_Pi));
	target_angle[sessionID-1] = ((angle < -90.0f) ? 90.0f : (angle > 90.0f ? 90.0f : angle));
	current_distance[sessionID-1] = VectorMath::magnitude(VectorMath::subtract(lead_pose, follower_pose).position);

	/* Vector math without involving z-axis rotation of the follower vehicle */
	/*
	Vector3f P2s = lead_pose.position - follower_pose.position;

	float alpha = std::atan2f(P2s.y(), P2s.x());
	float pitch, roll, yaw;
	VectorMath::toEulerianAngle(follower_pose.orientation, pitch, roll, yaw);

	steering_angle = -static_cast<float>((-alpha + yaw) * (180.0f / std::_Pi));
	*/
}

vector<string> split(const string &s, char delim) {
	vector<string> result;
	std::stringstream ss(s);
	string item;
	while (getline(ss, item, delim)) {
		result.push_back(std::move(item));
		item.clear();
	}
	return result;
}

void receiveControlsFromSimulation(TcpClient& tcp_client, CarRpcLibClient& follower_client,  int sessionID) {
	std::thread([&tcp_client, &follower_client, sessionID] {
		CarControls follower_controls;
		while (tcp_client.is_connected()) {
			try {
				char* msg = tcp_client.receive_message();
				if (msg != NULL) {
					string* str = new std::string(msg);
					vector<string> fields = split(*str, ',');
					if (fields.size() < 3) {
						msg = NULL;
						fields.clear();
						str->clear();
						continue;
					}
					if (std::atoi(fields[0].c_str()) == sessionID) {
						string name = fields[1];
						string value = fields[2];
						if (value.length() > 0) {
							float val = static_cast<float>(std::atof(value.c_str()));
							if (name == "Car.throttle")
								follower_controls.throttle = val;
							else if (name == "Car.brake")
								follower_controls.brake = val;
							else if (name == "Car.steering")
								follower_controls.steering = val;
						follower_client.setCarControls(follower_controls);
						}
					
					}
					msg = NULL;
					fields.clear();
					str->clear();
				}
			}
			catch (std::exception &e) {
				printf("error receiving message: %s\n", e.what());
			}
		}
	}).detach();
}

void sendControlsToSimulation(TcpClient& tcp_client, float follower_speed, float location_coeff, float weather_coeff, int sessionID) {
	if (tcp_client.is_connected()) {
		steering_sensitivity[sessionID-1] = fminf(fmaxf(1.0f / (follower_speed*3.6f), 0.0f), 1.0f);
		brake_track[sessionID-1] = ((follower_speed*3.6f)* weather_coeff*location_coeff) / 2.0f;
		std::ostringstream data_stream;
		data_stream << sessionID << ",Car.distance," << current_distance[sessionID-1] << "\n"
			<< sessionID << ",Car.min_distance," << fmaxf(brake_track[sessionID-1], MIN_STOP_DISTANCE) << "\n"
			<< sessionID << ",Car.target_angle," << target_angle[sessionID-1] << "\n"
			<< sessionID << ",Car.steer_sensitivity," << steering_sensitivity[sessionID-1] << "\n";
		
		tcp_client.send_message(data_stream.str().c_str());
	}
}

void establishPlatooning(TcpClient& tcp_client, CarRpcLibClient& lead_client, CarRpcLibClient& follower_client, int sessionID) {

	//float stop_distance = MIN_STOP_DISTANCE;

	float location_coeff = 1.0f;    //destination_coeff(t) = 2.0 minimum destination constant when driving outer-city, default t = 1.0
	float weather_coeff = 1.0f;   //weather_coeff(k) = 2.0 assumes wet road, default k = 1.0 for dry road
	std::chrono::steady_clock::time_point startTime;
	receiveControlsFromSimulation(tcp_client, follower_client, sessionID);
	while (lead_client.getConnectionState() == RpcLibClientBase::ConnectionState::Connected) {
		try {
			startTime = std::chrono::steady_clock::now();

			CarState lead_state = lead_client.getCarState();
			CarState follower_state = follower_client.getCarState();

			float lead_speed = lead_state.speed;
			float follower_speed = follower_state.speed;

			//location_coeff = follower_speed*3.6f >= 50.0f ? 2.0f : 1.0f;
			if (current_mode == Mode::image_processing) {
				//approximating distance to leading vehicle, due to image processing take some time
				//assumes that the speeds of the vehicles have not changed
				current_distance[sessionID-1] -= ((follower_speed - lead_speed) * (overallTime*0.001f));
			}
			else if (current_mode == Mode::vector_calculation) {
				processVectorCalculations(lead_state, follower_state, sessionID);
			}

			sendControlsToSimulation(tcp_client, follower_speed, location_coeff, weather_coeff, sessionID);
			
			overallTime = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now() - startTime).count();
		}
		catch (std::exception &e) {
			std::cout << e.what();
		}
	}
}

CarRpcLibClient* initPlatoonMember(const string& rpc_address, const uint16_t rpc_port, const string& tag_name, bool is_tag_name_regex, const int segmentationId, bool enable_api_control) {
	CarRpcLibClient* client = new CarRpcLibClient(rpc_address, rpc_port, RPC_TIMEOUT);
	client->confirmConnection();
	client->simSetSegmentationObjectID(tag_name, segmentationId, is_tag_name_regex);
	client->enableApiControl(enable_api_control);
	return client;
}



int main()
{

	CarRpcLibClient* suv1 = initPlatoonMember("localhost", 41451, "Suv1", false, 1, false);
	CarRpcLibClient* suv2 = initPlatoonMember("localhost", 41452, "Suv2", false, 2, true);
	CarRpcLibClient* suv3 = initPlatoonMember("localhost", 41453, "Suv3", false, 3, true);
	
	try {
		

			TcpClient* tcp_client = new TcpClient();
			tcp_client->start();
		std::thread([&tcp_client, &suv1, &suv2]() {
			establishPlatooning(*tcp_client, *suv1, *suv2, 1);
		}).detach();

		std::thread([&tcp_client, &suv2, &suv3]() {
			
			establishPlatooning(*tcp_client, *suv2, *suv3, 2);
		
		}).detach();



		if (current_mode == Mode::image_processing) {
			std::thread([&]() {
				processImages(*suv2, 1, 1);
			}).detach();
			std::thread([&]() {
				processImages(*suv3, 2, 2);
			}).detach();

		}


		std::cin.get();
			tcp_client->stop();
			tcp_client->~TcpStatechartClient();
			delete tcp_client;
	
	}
	catch (std::exception & e) {
		std::cerr << e.what();
	}

	delete suv1;
	suv1 = NULL;
	delete suv2;
	suv2 = NULL;
	delete suv3;
	suv3 = NULL;

	return 0;
}
