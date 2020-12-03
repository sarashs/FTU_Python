import serial.tools.list_ports
ports = serial.tools.list_ports.comports()

print("Available Serial Ports : \n")
for port, desc, hwid in sorted(ports):
        print("{}: {}".format(port, desc))