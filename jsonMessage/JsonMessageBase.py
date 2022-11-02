import json

class JsonMessageBase:

    raw_message:str
    dict_write:dict

    def __init__(self):
        self.raw_message="null"
        self.dict_write={}

    def to_json(self):
        return json.dumps(self.dict_write)

    def readData(self,json):
        self.raw_message=json