import socket

IP='0.0.0.0'
PORT=9999
BUFSIZE=4096

class Server:
    def __init__(self,ip=IP,port=PORT):
        self.server=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        socket.SO_REUSEPORT=15
        #self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.server.settimeout(10)
        self.server.bind((ip,port))
        self.server.listen(1)

    def receive(self,callback):
        try:
            client,_ = self.server.accept()

            data=client.recv(BUFSIZE)
            string=data.decode('utf-8')
            output=callback(string)
            if output:
                data = output.encode('utf-8')
                client.sendall(data+b' '*(BUFSIZE-len(data)))
            client.close()
            
        except Exception as err:
            print(err)
    
    def receive_long(self,callback):
        try:
            client,_ = self.server.accept()

            while True:
                data=client.recv(BUFSIZE)
                if len(data)==0:
                    print('close')
                    break
                string=data.decode('utf-8')
                output=callback(string)
                if output:
                    data = output.encode('utf-8')
                    client.sendall(data+b' '*(BUFSIZE-len(data)))
            
            client.close()
            
        except Exception as err:
            print(err)

    def close(self):
        self.server.close()

class Server_udp():
    def __init__(self,ip=IP,port=PORT):
        self.server=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        socket.SO_REUSEPORT=15
        #self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.server.settimeout(10)
        self.server.bind((ip,port))
    
    def receive(self,callback,dst=None):
        try:
            data=self.server.recv(BUFSIZE)
            string=data.decode('utf-8')
            output=callback(string)
            if output and dst:
                data = output.encode('utf-8')
                self.server.sendto(data+b' '*(BUFSIZE-len(data)),dst)
            
        except Exception as err:
            print(err)

    def receiveOnThread(self,callback,dst=None):
        import threading
        def run():
            while(True):
                self.receive(callback,dst)
        t=threading.Thread(target = run)
        t.daemon = True
        t.start()
        
if __name__=="__main__":
    from time import sleep
    server=Server_udp()
    def callback(output):
        print(output)
    server.receiveOnThread(callback)
    while True:
        sleep(1)