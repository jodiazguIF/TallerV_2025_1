import socket
HOST = "192.168.1.43"
PORT = 12345

mySocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mySocket.settimeout(5.0)
print("UDP Client started. Ingresa un PID (motor, P, I, D) or \'quit\' to exit.")
while (True):
    PID = input("Ingresa un PID: ")
    if PID.lower() == "quit":
        print("Exiting...")
        break
    sendPID = PID.encode('utf-8')
    mySocket.sendto(sendPID, (HOST, PORT))
    print("PID enviado:", PID, "a", HOST, ":", PORT)
    