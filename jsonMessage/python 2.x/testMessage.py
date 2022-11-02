import json
from jsonMessageBase import *

class TestMessage(jsonMessageBase):
    timestamp=0
    pos3=[0,0,0]

    def __init__(self):
        self.timestamp=-1
        self.pos3=[0.0,0.0,0.0]
        self.dict_write={"timestamp":self.timestamp,
                              "pos3":self.pos3}

    def to_json(self):
        self.dict_write["timestamp"]=self.timestamp
        self.dict_write["pos3"]=self.pos3
        return super(TestMessage,self).to_json()

    def readData(self, json_string):
        super(TestMessage,self).readData(json_string)
        data=json.loads(json_string)
        self.timestamp=data["timestamp"]
        self.pos3=data["pos3"]
