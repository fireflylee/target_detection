#ifndef WAYPOINT_MESSAGE_HPP_
#define WAYPOINT_MESSAGE_HPP_

#include <iostream>
#include <jsonMessageBase.hpp>
#include <json/json.h>

using namespace std;

class WaypointMessage : public JsonMessageBase
{
public:
    bool isStart=false;
    bool isPause=false;
    bool isContinue=false;

    double x=0;
    double y=0;
    int height=0;
    int speed=0;

    virtual string to_json()
    {
        if(!isStart && !isContinue && !isPause)
        {
            root_write["command"]="update";
            root_write["lngs"][0]=(int)(this->x/111000*1e7);
            root_write["lats"][0]=(int)(this->y/111000*1e7);
            root_write["heis"][0]=this->height;
            root_write["spds"][0]=this->speed;
        }
        else
        {
            if(isStart)
            {
                root_write["command"]="start";
            }
            else if(isContinue)
            {
                root_write["command"]="continue";
            }
            else if(isPause)
            {
                root_write["command"]="pause";
            }
            root_write["lngs"]=NULL;
            root_write["lats"]=NULL;
            root_write["heis"]=NULL;
            root_write["spds"]=NULL;
        }
        return fastWriter.write(root_write);
    }
};

#endif