//
// Created by Codetonsen on 10/29/2024.
//

#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <mutex>
#include <string>
#include <thread>

#include "simple_socket/TCPSocket.hpp"

class TCPCommunication {
public:
    TCPCommunication(int port);
    ~TCPCommunication();
    bool isConnected() const;
    void sendMessage(const std::string& message);
    void receiveMessages();

private:
    simple_socket::TCPServer server;
    std::unique_ptr<simple_socket::SimpleConnection> connection;
    std::thread receiveThread;
    std::mutex connectionMutex;
    bool connected;

    void listenForConnections();
    void handleDisconnect();
};


#endif //COMMUNICATION_H
