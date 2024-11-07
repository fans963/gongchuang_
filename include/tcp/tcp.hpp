#pragma once

#include <arpa/inet.h>
#include <atomic>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <thread>

class TCPServer {
public:
    explicit TCPServer(const std::string& host = "127.0.0.1", int port = 9000)
        : host(host)
        , port(port)
        , server_fd(-1) {}

    virtual ~TCPServer() { stop(); }

    std::atomic<bool> tcp_flag = std::atomic<bool>(false);

    bool tcp_start() {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd == -1) {
            std::cerr << "无法创建socket" << std::endl;
            return false;
        }

        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family      = AF_INET;
        server_addr.sin_addr.s_addr = inet_addr(host.c_str());
        server_addr.sin_port        = htons(port);

        if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "绑定失败" << std::endl;
            return false;
        }

        if (listen(server_fd, 5) < 0) {
            std::cerr << "监听失败" << std::endl;
            return false;
        }

        std::cout << "服务器启动，监听 " << host << ":" << port << std::endl;

        while (true) {
            int client_fd = accept(server_fd, nullptr, nullptr);
            if (client_fd >= 0) {
                std::cout << "接受到连接" << std::endl;
                std::thread(&TCPServer::handle_client, this, client_fd).detach();
                tcp_flag.store(true);
            }
        }
        return true;
    }

    void stop() {
        if (server_fd != -1) {
            close(server_fd);
            server_fd = -1;
            std::cout << "服务器已停止" << std::endl;
        }
    }

private:
    virtual void handle_client(int client_fd) = 0;

    std::string host;
    int port;
    int server_fd;
};
