import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
import random
import serial
import argparse

#initialize serial port
ser = serial.Serial()
ser.port = 'COM3' #Arduino serial port
ser.baudrate = 115200
ser.timeout = 10 #specify timeout when using readline()
ser.open()
if ser.is_open==True:
	print("\nAll right, serial port now open. Configuration:\n")
	print(ser, "\n") #print serial parameters

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ns = [] #store trials here (n)
xs = [] 
ys = [] 
zs = [] 

# This function is called periodically from FuncAnimation
def animate(n, xs, ys, zs):

    #Aquire and parse data from serial port
    line=ser.readline()      #ascii
    line_as_list = line.split(b',')
    x = float(line_as_list[0])
    y = float(line_as_list[1])
    z = float(line_as_list[2])
    n_str = line_as_list[3]
    n_str_list = n_str.split(b'\n')
    n = int(n_str_list[0])
	
	# Add x and y to lists
    xs.append(x)
    ys.append(y)
    zs.append(z)
    ns.append(n)

    # Limit x and y lists to 20 items
    #xs = xs[-20:]
    #ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(ns, xs, label="x")
    ax.plot(ns, ys, label="y")
    ax.plot(ns, zs, label="z")

    # Format plot
    #plt.xticks(rotation=45, ha='right')
    #plt.subplots_adjust(bottom=0.30)
    #plt.title('This is how I roll...')
    #plt.ylabel('Relative frequency')
    plt.legend()
    plt.axis([ns[0], ns[len(ns)-1], None, None]) #Use for arbitrary number of trials
    #plt.axis([1, 100, 0, 1.1]) #Use for 100 trial demo

    if len(xs)>25:
        del xs[0] 
        del ys[0]
        del zs[0]
        del ns[0]


# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys, zs), interval=1)
plt.show()
