#ifndef JSON_READER_HPP_
#define JSON_READER_HPP_

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

#define READER_SIZE 2048

class JsonReader
{

public:
    bool isNew=false;

    JsonReader(JsonMessageBase& data, int port):data(data)
    {
        init(port);
    }
    
    virtual ~JsonReader()
    {
        isEnding = true;
        if(sockfd>0)
        {
            close(sockfd);
        }
    }

    void readData()
    {
        if(isNew)
        {
            mlock.lock();
            std::string json = msg;
            isNew=false;
            mlock.unlock();
            data.read(json);
        }
        else
            return;
    }

private:
    std::string msg;
    bool isEnding = false;
    pthread_t run_pthread;
    std::mutex mlock;
    int sockfd=0;
    JsonMessageBase& data;

    void init(int port)
    {
        sockfd=socket(AF_INET,SOCK_DGRAM,0);
        if(sockfd<=0)
        {
            throw "socket error";
            return;
        }
	    int flag = 1;
	    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag));

        struct sockaddr_in local;
        memset(&local,'\n',sizeof(local));
        local.sin_family=AF_INET;
        local.sin_port=htons(port);
        local.sin_addr.s_addr=INADDR_ANY;

        int nRecvBuf = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&nRecvBuf, sizeof(int));

        if(bind(sockfd,(struct sockaddr*)&local,sizeof(local))<0)
        {
            throw "bind error";
            return;
        }

        msg = "null";
        pthread_create(&run_pthread, NULL, &JsonReader::runByThread, this);
    }

    static void* runByThread(void* self)
    {
        return static_cast<JsonReader*>(self)->run();
    }

    void* run()
    {
        char buff[READER_SIZE]={0};
        sockaddr_in client;
        socklen_t len=sizeof(client);
        while (!isEnding)
        {
            memset(buff, 0, sizeof(buff));
            ssize_t size=recvfrom(sockfd,buff,sizeof(buff)-1,0,(struct sockaddr*)&client,&len);
            if(size>0)
            {
                buff[size]='\0';
                do
                {
                    if (mlock.try_lock())
                    {
                        msg=buff;
                        isNew=true;
                        mlock.unlock();
                        break;
                    }
                    usleep(1000);
                } while (1);
            }
        }
        return nullptr;
    }

};

#endif