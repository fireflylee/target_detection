from jsonMessage.NTPMessage import NTPMessage
from time import sleep

# "PUBLISHER" / 'SUBSCRIBER'

MODE="PUBLISHER"

print(MODE)
print()

if MODE=='PUBLISHER':
    from jsonMessage.JsonSender import JsonSender

    ntpm=NTPMessage()
    js=JsonSender(ntpm,"192.168.42.255",5873)

    while True:
        js.sendData()
        print('Tick:'+str(ntpm.time))
        sleep(10)

elif MODE=='SUBSCRIBER':
    from jsonMessage.JsonReader import JsonReader
    import os

    def sudoCMD(command,password):
        str = os.system('echo %s | sudo -S %s' % (password,command))
        print(str)
    
    def CMD(command):
        str = os.system(command)
        print(str)

    def callback(time):
        CMD('timedatectl set-ntp false')
        CMD('timedatectl set-time '+ '\"' + time.strftime("%Y-%m-%d %H:%M:%S") + '\"')
        CMD('timedatectl set-local-rtc 1')
        print("Tock:"+time.strftime("%Y-%m-%d %H:%M:%S"))
        print()

    ntpm=NTPMessage(callback)
    js=JsonReader(ntpm,5873)

    while True:
        sleep(10)
