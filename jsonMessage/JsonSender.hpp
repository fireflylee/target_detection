#ifndef JSON_SENDER_HPP_
#define JSON_SENDER_HPP_

#include <mutex>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <string>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <jsonMessageBase.hpp>

#define SENDER_SIZE 2048

class JsonSender
{

public:

    JsonSender(JsonMessageBase& data, const string& ip, int port):data(data)
    {
        init(ip,port);
    }

    virtual ~JsonSender()
    {
        if(sockfd>0)
        {
            close(sockfd);
        }
    }

    void sendData()
    {
        msg=data.to_json();

        strcpy(buff, msg.c_str());
        sendto(sockfd, buff, msg.size()+1, 0, (struct sockaddr*)&serveraddr, len);
    }

private:

    std::string msg;
    int sockfd=0;
    JsonMessageBase& data;
    char buff[SENDER_SIZE]={0};
    socklen_t len;
    struct sockaddr_in serveraddr;

    void init(const string& ip, int port)
    {
        sockfd=socket(AF_INET,SOCK_DGRAM,0);
        if(sockfd<=0)
        {
            throw "socket error";
            return;
        }
        int flag = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &flag, sizeof(flag));

        len = sizeof(struct sockaddr_in);
        memset(&serveraddr, 0, len);

        serveraddr.sin_family = AF_INET;
        serveraddr.sin_port = htons(port);
        serveraddr.sin_addr.s_addr = inet_addr(ip.c_str());
    }
    
};

#endif