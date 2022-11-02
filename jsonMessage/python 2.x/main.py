from JsonReader import *
from JsonSender import *
from testMessage import *
from time import sleep

tm=TestMessage()
jr=JsonReader(tm,22222)

tm_pub=TestMessage()
js=JsonSender(tm_pub,"127.255.255.255",22222)

while True:
    tm_pub.timestamp+=1
    tm_pub.pos3[0]=tm_pub.timestamp
    print(tm_pub.to_json())
    print(tm_pub.timestamp)
    print(tm_pub.pos3)
    js.sendData()

    sleep(0.01)

    print(tm.timestamp)
    print(tm.pos3)