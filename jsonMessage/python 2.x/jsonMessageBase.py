import json

class jsonMessageBase(object):

    raw_message=""
    dict_write={}

    def __init__(self):
        self.raw_message="null"
        self.dict_write={}

    def to_json(self):
        return json.dumps(self.dict_write)

    def readData(self,json):
        self.raw_message=json
