#ifndef TRACK_MESSAGE_HPP_
#define TRACK_MESSAGE_HPP_

#include <iostream>
#include <jsonMessageBase.hpp>
#include <json/json.h>

using namespace std;

class trackMessage : public JsonMessageBase
{
public:
    bool isTrack=false;
    double x=0,y=0;
    int trackId=-1;

    virtual string to_json()
    {
        root_write["isTrack"]=isTrack;
        root_write["x"]=x;
        root_write["y"]=y;
        root_write["trackId"]=trackId;
        return fastWriter.write(root_write);
    }

    virtual void read(const string& json)
    {
        raw_message=json;
        reader.parse(json, root_read);

        isTrack = root_read["isTrack"].asBool();
        x = root_read["x"].asDouble();
        y = root_read["y"].asDouble();
        trackId = root_read["trackId"].asInt();
    }
};

#endif