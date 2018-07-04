#pragma once

// size of our buffer
#define DEFAULT_BUFLEN 1024
// port to connect sockets through 
#define DEFAULT_PORT "12345"
#define DEFAULT_ADDRESS "localhost"
// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")
#include <string>
#include <cstdlib>

class TcpStatechartClient
{
public:
	TcpStatechartClient();
	~TcpStatechartClient();

	//Starts a TCP client and connects it to a server
	bool start();
	//Stops TCP client and disconnects it from the server
	bool stop();
	// Send message to server
	bool send_message(const char* msg);
	// Receive message from server
	char* receive_message();

	SOCKET connection_socket;
	char received_message[DEFAULT_BUFLEN];
	bool is_connected();

private:
	bool connected;

};

