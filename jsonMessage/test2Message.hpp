#ifndef TEST2_MESSAGE_HPP_
#define TEST2_MESSAGE_HPP_

#include <iostream>
#include <jsonMessageBase.hpp>
#include <json/json.h>

#include <vector>

using namespace std;

class test2Message : public JsonMessageBase
{
public:
    int timestamp=-1;
    vector<double> pos3={0,0,0};

    virtual string to_json()
    {
        root_write["timestamp"]=Json::Value(timestamp);
        for(int i=0;i<pos3.size();i++)
            root_write["pos3"][i]=Json::Value(pos3[i]);
        return fastWriter.write(root_write);
    }

    virtual void read(const string& json)
    {
        raw_message=json;
        reader.parse(json, root_read);
        timestamp = root_read["timestamp"].asInt();
        for(int i=0;i<pos3.size();i++)
            pos3[i]=root_read["pos3"][i].asDouble();
    }
};

#endif