#pragma once
#include "ImageProcessor.h"
#include <future>
#include <iostream>
#include <chrono>
#include <cstdlib>

using namespace msr::airlib;



ImageProcessor::ImageProcessor()
{

}

ImageProcessor::ImageProcessor(int id) : objectId(id)
{
}

ImageProcessor::~ImageProcessor()
{
}

void ImageProcessor::findPositionForColor(ImageResponse& image_info, int(&rgb)[3]) {
	if (rgb[0] != -1 && rgb[1] != -1 && rgb[2] != -1) {
		const uint8_t* image_data = image_info.image_data_uint8.data();
		int arrIndex = 0;
		minX = INT_MAX;
		minY = INT_MAX;
		maxX = INT_MIN;
		maxY = INT_MIN;
		for (int y = 0; y < image_info.height; y++) {
			for (int x = 0; x < image_info.width; x++) {

				uint8_t r = image_data[arrIndex++];
				uint8_t g = image_data[arrIndex++];
				uint8_t b = image_data[arrIndex++];
				arrIndex++;

				if (r == rgb[0] && g == rgb[1] && b == rgb[2]) {
					if (x < minX) minX = x;
					if (x > maxX) maxX = x;
					if (y > maxY) maxY = y;
					if (y < minY) minY = y;
				}
			}
		}
	}
}

float ImageProcessor::calculateSteeringAngle(CarRpcLibClient& client) {
	const vector<ImageRequest> segmentationRequest = { ImageRequest(0, ImageType::Segmentation,false,false) };
	const vector<ImageResponse>& response = client.simGetImages(segmentationRequest);
	int rgb[] = { -1,-1,-1 };
	for (ImageResponse image_info : response) {
		if (image_info.compress == false && image_info.image_type == ImageType::Segmentation) {

			switch (ImageProcessor::objectId) {
			case 1:
				rgb[0] = 153;
				rgb[1] = 108;
				rgb[2] = 6;
				break;
			case 2:
				rgb[0] = 112;
				rgb[1] = 105;
				rgb[2] = 191;
				break;
			case 3:
				rgb[0] = 89;
				rgb[1] = 121;
				rgb[2] = 72;
				break;
			case 4:
				rgb[0] = 190;
				rgb[1] = 225;
				rgb[2] = 64;
				break;
			default:
				break;
			}
			findPositionForColor(image_info, rgb);

			midX = image_info.width / 2;
			midY = image_info.height / 2;

			int centerX = (maxX + minX) / 2;
			int centerY = (maxY + minY) / 2;

			Vector2f cameraCenter = Vector2f(midX, image_info.height);
			vehicleCenter = Vector2f(centerX, centerY);
			boundingBox = new int[4]{ minX,maxX,minY,maxY };
			angle = static_cast<float>(atan2f(cameraCenter.y() - vehicleCenter.y(), cameraCenter.x() - vehicleCenter.x()) * (180.0f / std::_Pi)) - 90.0f;
		}
		else if (image_info.compress) { 
			std::ofstream file("C:\\Users\\rudi\\Desktop\\AirSimImages\\"+ std::to_string(image_info.time_stamp) + ".png", std::ios::binary);
			file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
			file.close();
		}
	}
	return angle;
}

float ImageProcessor::calculateDistance(CarRpcLibClient& client) {
	const vector<ImageRequest> depthRequest = { ImageRequest(0,ImageType::DepthVis,true,false) };
	const vector<ImageResponse>& response = client.simGetImages(depthRequest);
	float minDepth = FLT_MAX;
	//int arrIndex = 0;
	for (ImageResponse image_info : response) {
		if (image_info.pixels_as_float && image_info.compress == false && image_info.image_type == ImageType::DepthVis) {
			const float* depthData = image_info.image_data_float.data();
			for (int y = boundingBox[2]; y < boundingBox[3]; y++) {
				for (int x = boundingBox[0]; x < boundingBox[1]; x++) {

					float grayscale = depthData[y + (x*image_info.height)];
					if (grayscale < minDepth) {
						minDepth = grayscale;
						distance = minDepth * 100;
					}

					//arrIndex++;
				}
			}
		}
		else if (image_info.compress) {
			Utils::writePfmFile(image_info.image_data_float.data(), image_info.width, image_info.height,
				"C:\\Users\\rudi\\Desktop\\AirSimImages\\" + std::to_string(image_info.time_stamp) + ".pfm");
		}
	}
	return distance;
}