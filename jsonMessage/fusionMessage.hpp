#include <iostream>
#include <jsonMessageBase.hpp>
#include <json/json.h>

#include <vector>

using namespace std;

#define UAV_NUM 12

class FusionMessage : public JsonMessageBase
{
public:
    vector<int> indexList;
    vector<vector<double>> posList;
    vector<vector<double>> velocityList;
    vector<string> stateList;
    vector<vector<int>> fromList;
    vector<vector<int>> oriTargetIndexList;

    FusionMessage()
    {
        clear(0);
    }

    virtual void read(const string& json)
    {
        raw_message=json;
        reader.parse(json, root_read);

        clear(root_read["fusions"].size());

        for(int i=0;i<root_read["fusions"].size();i++)
        {
            indexList.push_back(root_read["fusions"][i]["id"].asInt());
            velocityList.push_back(vector<double>{root_read["fusions"][i]["vx"].asDouble(),
                                                  root_read["fusions"][i]["vy"].asDouble());
            posList.push_back(vector<double>{root_read["fusions"][i]["x"].asDouble(),
                                             root_read["fusions"][i]["y"].asDouble(),
                                             root_read["fusions"][i]["z"].asDouble()});
            stateList.push_back(root_read["fusions"][i]["state"].asString());
            for(int j=0;j<root_read["fusions"][i]["from"].size();j++)
            {
                fromList[i].push_back(root_read["fusions"][i]["from"][j][(int)0].asInt());
                oriTargetIndexList[i][fromList[i].back()-1]=(root_read["fusions"][i]["from"][j][1].asInt());
            }
        }
    }

    int getUavTargetId(int fusionId, int uavId)    //start from 1
    {
        for(int i=0;i<indexList.size();i++)
        {
            if(indexList[i]==fusionId)
            {
                return oriTargetIndexList[i][uavId-1];
            }
        }
        return -1;
    }

    bool getPredictPos(int fusionId, double deltaT,double& x, double& y)
    {
        for(int i=0;i<indexList.size();i++)
        {
            if(indexList[i]==fusionId)
            {
                x=posList[i][0]+velocityList[i][0]*deltaT
                y=posList[i][1]+velocityList[i][1]*deltaT
                return true;
            }
        }
        return false;
    }

    void fillVector(std::vector<std::vector<int>>& vec, std::vector<std::vector<float>>& vec_pos)
    {
        vec.clear();
        vec_pos.clear();

        int maxIndex=-1;
        for(int index : indexList)
        {
            if(index>maxIndex)
                maxIndex=index;
        }

        for(int i=1;i<=maxIndex;i++)
        {
            vec.push_back(std::vector<int>());
            vec_pos.push_back(std::vector<float>());
            for(int j=0;j<indexList.size();j++)
            {
                if(indexList[j]==i)
                {
                    for(int k=0;k<fromList[j].size();k++)
                    {
                        vec.back().push_back(fromList[j][k]);
                    }
                    vec_pos.back().push_back(posList[j][0]);
                    vec_pos.back().push_back(posList[j][1]);
                    vec_pos.back().push_back(posList[j][2]);
                    break;
                }
            }
        }
    }

    void fillVector2(std::vector<std::vector<int>>& vec, std::vector<std::vector<float>>& vec_pos)
    {
        for(int i=0;i<UAV_NUM;i++)
            vec.at(i).clear();
        vec_pos.clear();

        int maxIndex=-1;
        for(int index : indexList)
        {
            if(index>maxIndex)
                maxIndex=index;
        }

        for(int i=1;i<=maxIndex;i++)
        {
            vec_pos.push_back(std::vector<float>());
            for(int j=0;j<indexList.size();j++)
            {
                if(indexList[j]==i)
                {
                    for(int k=0;k<fromList[j].size();k++)
                    {
                        vec.at(fromList[j][k])-1).push_back(i);
                    }
                    vec_pos.back().push_back(posList[j][0]);
                    vec_pos.back().push_back(posList[j][1]);
                    vec_pos.back().push_back(posList[j][2]);
                    break;
                }
            }
        }
    }

private:
    void clear(int size)
    {
        indexList.clear();
        posList.clear();
        velocityList.clear();
        stateList.clear();
        fromList.clear();
        oriTargetIndexList.clear();
        for(int i=0;i<size;i++)
        {
            oriTargetIndexList.push_back(vector<int>());
            fromList.push_back(vector<int>());
            for(int j=0;j<UAV_NUM;j++)
                oriTargetIndexList.back().push_back(-1);
        }
    }
};