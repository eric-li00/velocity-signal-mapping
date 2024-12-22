import socket
from DataInterfacePython import *

def ModelStart(userData):
    # Define the bus data format statement of the vehicle
    EGO_FORMAT = "time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d"
    # Construct the vehicle bus reader-writer
    userData["ego"] = BusAccessor(userData["busId"], "ego", EGO_FORMAT)
    # Create a socket and assign the socket object to userData["client_sock"]      IPv4 address and UDP protocol
    userData["server_address"] = '192.168.1.106'
    userData["server_port"] = 17000
    userData["client_sock"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  

def ModelOutput(userData):

    # Define the main vehicle bus format, data reading, processing, and sending.
    t,x,y,z,yaw,pitch,roll,speed1 =userData["ego"].readHeader()
    speed2=round(speed1)
    speed=str(speed2)
    userData["client_sock"].sendto(speed.encode(),(userData["server_address"],userData["server_port"]))
    print(t,x,y,z,yaw,pitch,roll,speed)
    

def ModelTerminate(userData):
    userData["client_sock"].close()