import json
from .JsonMessageBase import *

class TrackMessage(JsonMessageBase):
    #定义数据内容
    isTrack:bool
    x:float
    y:float
    id:int

    def __init__(self):
        super().__init__()
        #数据初始化
        self.isTrack=False
        self.x=0
        self.y=0
        self.id=0
        #构造字典
        self.dict_write={"isTrack":self.isTrack,"x":self.x,"y":self.y,"trackId":self.id}

    def to_json(self):
        #数据更新
        self.dict_write["isTrack"]=self.isTrack
        self.dict_write["x"]=self.x
        self.dict_write["y"]=self.y
        self.dict_write["trackId"]=self.id
        #保留
        return super().to_json()

    def readData(self, json_string):
        #保留
        super().readData(json_string)
        data=json.loads(json_string)
        #拆包
        self.isTrack=data["isTrack"]
        self.x=data["x"]
        self.y=data["y"]
        self.id=data["trackId"]