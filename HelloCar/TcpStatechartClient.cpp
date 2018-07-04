#pragma once
#include <WinSock2.h>

#include <WS2tcpip.h>
#include "TcpStatechartClient.h"
#include <string>
#include <cstdlib>
#include <thread>
using namespace std;
TcpStatechartClient::TcpStatechartClient()
{
	connection_socket = INVALID_SOCKET;
}

bool TcpStatechartClient::start() {
	WSADATA wsaData;

	// Initialize Winsock
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0)
	{
		printf("WSAStartup failed: %d\n", iResult);
		return false;
	}

	struct addrinfo	*result = NULL,
		*ptr = NULL,
		hints;

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo(DEFAULT_ADDRESS, DEFAULT_PORT, &hints, &result);
	if (iResult != 0)
	{
		printf("getaddrinfo failed: %d\n", iResult);
		WSACleanup();
		return false;
	}

	ptr = result;

	// Create a SOCKET for connecting to server
	connection_socket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);

	if (connection_socket == INVALID_SOCKET)
	{
		printf("Error at socket(): %d\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return false;
	}

	// Connect to server
	iResult = connect(connection_socket, ptr->ai_addr, (int)ptr->ai_addrlen);

	if (iResult == SOCKET_ERROR)
	{
		closesocket(connection_socket);
		connection_socket = INVALID_SOCKET;
	}

	freeaddrinfo(result);

	if (connection_socket == INVALID_SOCKET)
	{
		printf("Unable to connect to server!\n");
		WSACleanup();
		std::this_thread::sleep_for(std::chrono::duration<double>(5.0));
		printf("Try to reconnect...\n");
		start();
		return false;
	}
	else {
		printf("Successfully connected to %s:%s\n", DEFAULT_ADDRESS ,DEFAULT_PORT);
	}
	// Set the mode of the socket to be nonblocking
	u_long iMode = 1;
	iResult = ioctlsocket(connection_socket, FIONBIO, &iMode);
	if (iResult == SOCKET_ERROR)
	{
	printf("ioctlsocket failed with error: %d\n", WSAGetLastError());
		closesocket(connection_socket);
		WSACleanup();
		return false;
	}

	return true;
};

// Free the resouces
bool TcpStatechartClient::stop() {
	int iResult = shutdown(connection_socket, SD_SEND);

	if (iResult == SOCKET_ERROR)
	{
		printf("shutdown failed: %d\n", WSAGetLastError());
		return false;
	}

	closesocket(connection_socket);
	WSACleanup();
	return true;
};

// Send message to server
bool TcpStatechartClient::send_message(const char* msg)
{
	std::string str(msg);
	int iResult = send(connection_socket, msg, static_cast<int>(str.length()), 0);
	if (iResult == SOCKET_ERROR)
	{
		printf("send failed: %d\n", WSAGetLastError());
		//Stop();
		return false;
	}

	return true;
};

// Receive message from server
char* TcpStatechartClient::receive_message()
{
	char recvbuf[DEFAULT_BUFLEN];

		int iResult = recv(connection_socket, recvbuf, DEFAULT_BUFLEN, 0);

		if (iResult > 0)
		{
			memset(&received_message, 0, sizeof(received_message));

			strncpy_s(received_message, recvbuf, iResult);
			return received_message;
		}
	

	return NULL;
}

bool TcpStatechartClient::is_connected() {
	return connection_socket != INVALID_SOCKET;
}

TcpStatechartClient::~TcpStatechartClient()
{
	connection_socket = INVALID_SOCKET;
}

