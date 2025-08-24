import serial
import datetime

arduino_port = "/dev/ttyACM2" 
baud = 9600
current_time = datetime.datetime.now()
fileName=f'weight-data-{current_time}.csv'
samples = 1000
print_labels = False

ser = serial.Serial(arduino_port, baud)
print("Connected to Arduino port:" + arduino_port)




with open(fileName, "w") as file:
    print("Created file")
    line = 0
    while line <= samples:
        if print_labels:
            if line==0:
                print("Printing column headers")
            else:
                print("Line" + str(line) + ":writing...")
        getData = str(ser.readline())
        data = getData[2:][:-5]
        print(data)

        file = open(fileName, "a")
        file.write(data + "\n")
        line += 1

print("Data collection complete")
