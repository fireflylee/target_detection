from jsonMessage.TrackMessage import TrackMessage
from jsonMessage.JsonSender import JsonSender

ip=input("IP:")
print()

while True:
    msg=TrackMessage()
    id=int(input("Id:"))
    js=JsonSender(msg,ip,40000+id)
    msg.isTrack=int(input("IsTrack:"))==1
    msg.id=int(input("TrackId:"))
    msg.x=float(input("x:"))
    msg.y=float(input("y:"))
    js.sendData()
    print()
    