#ifndef JSON_MESSAGE_BASE_HPP_
#define JSON_MESSAGE_BASE_HPP_

#include <iostream>
#include <json/json.h>

using namespace std;

class JsonMessageBase
{
public:
    virtual string to_json()
    {
        return "null";
    }

    virtual void read(const string& json)
    {
        return;
    }
    
protected:
    string raw_message="null";
    Json::Reader reader;
    Json::Value root_read,root_write;
    Json::FastWriter fastWriter;
    
};

#endif