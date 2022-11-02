import json
from datetime import datetime
from .JsonMessageBase import *

class NTPMessage(JsonMessageBase):
    time:datetime

    def __init__(self,callback=None):
        super().__init__()
        self.updateTime()
        self.callback=callback
        self.dict_write={"time":self.time.strftime("%Y-%m-%d %H:%M:%S")}

    def updateTime(self):
        self.time=datetime.now()

    def to_json(self):
        self.updateTime()
        self.dict_write["time"]=self.time.strftime("%Y-%m-%d %H:%M:%S")
        return super().to_json()

    def readData(self, json_string):
        super().readData(json_string)
        data=json.loads(json_string)
        self.time=datetime.strptime(data['time'], format("%Y-%m-%d %H:%M:%S"))
        if self.callback:
            self.callback(self.time)
