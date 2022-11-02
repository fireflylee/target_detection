#ifndef TEST_MESSAGE_HPP_
#define TEST_MESSAGE_HPP_

#include <iostream>
#include <jsonMessageBase.hpp>
#include <json/json.h>

using namespace std;

class testMessage : public JsonMessageBase
{
public:
    //自定义数据内容
    int id=0;
    string name="test";
    string value="";

    //实现从数据封装到json
    virtual string to_json()
    {
        //---数据封装过程---
        root_write["id"]=id;
        root_write["name"]=name;
        root_write["value"]=value;
        //-----------------

        //保留
        return fastWriter.write(root_write);
    }

    //实现从json读取数据
    virtual void read(const string& json)
    {
        //保留
        raw_message=json;
        reader.parse(json, root_read);

        //---数据抽取过程---
        name = root_read["name"].asString();
        id = root_read["id"].asInt();
        value = root_read["value"].asString();
        //-----------------
    }
};

#endif