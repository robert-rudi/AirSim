#pragma once
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include <iostream>


typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;
typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;
typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
typedef Eigen::Vector2f Vector2f;

class ImageProcessor {
public:
	ImageProcessor();
	ImageProcessor(int id);
	~ImageProcessor();

	float calculateSteeringAngle(msr::airlib::CarRpcLibClient& client);
	float calculateDistance(msr::airlib::CarRpcLibClient& client);
	void findPositionForColor(ImageResponse& image_info, int(&rgb)[3]);
	float getAngle() { return angle; };
	float getDistance() { return distance; };
	void setAngle(float a) { angle = a; };
	void setDistance(float d) { distance = d; };
private:
	Vector2f vehicleCenter;
	int minX = -1;
	int minY = -1;
	int maxX = -1;
	int maxY = -1;
	int midX = -1;
	int midY = -1;
	int objectId = -1;
	float angle = 90.0f;
	float distance = 0.0f;
	int* boundingBox;
};

