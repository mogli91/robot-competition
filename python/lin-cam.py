# This script is a copy of Stefan's gripperIRVisualiser but a different algorithm for line detection is used (hough transform).
#!/usr/bin/python
import sys, os
import time
import serial
import numpy as np
from matplotlib import cm
from matplotlib.pylab import subplots,close
import matplotlib.pyplot as plt

# for key board interrupts
import threading
import termios, fcntl

# serial port to connect to
port = '/dev/ttyACM1'
speed = 115200
serialObj = None

# linear camera stuff
num_px = 612
scale = 1.0/255
radius = 5.0 
angle_off = 0 #2*np.pi/3
circle1=plt.Circle((0,0), radius, color='r')
angles = np.linspace(0, 2*np.pi, num_px, endpoint=False) 
data = np.ones(num_px)
noise = np.random.normal(0,1,num_px)
radii_scaled = data * scale + noise
x = np.cos(angles) * radii_scaled
y = np.sin(angles) * radii_scaled

#compass stuff
compass = 0

# setup animated plot
fig,ax = subplots(1)
plt_x_axis = ax.plot(np.arange(-10, 11), np.zeros(21), linewidth=1.0, color='b')[0]
plt_y_axis = ax.plot(np.zeros(21), np.arange(-10, 11), linewidth=1.0, color='b')[0]
plt_data = ax.plot([], [], linewidth=1.0, color='m')[0]
plt_data_peak00 = ax.plot([0, radius * 2], [0, radius * 2], linewidth=1.0, color='r')[0]
plt_data_peak10 = ax.plot([0, -radius * 2], [0, radius * 2], linewidth=1.0, color='g')[0]
plt_data_peak11 = ax.plot([0, -radius * 2], [0, -radius * 2], linewidth=1.0, color='b')[0]
plt_data_peak01 = ax.plot([0, radius * 2], [0, -radius * 2], linewidth=1.0, color='y')[0]
peak00 = 0
peak10 = 0
peak11 = 0
peak01 = 0

ax.add_artist(circle1)
plt_direction = ax.arrow(-radius/2, -radius/2, radius, radius, width=0.03)
#plt_detection2 = ax.plot([],[],linewidth=10.0,alpha=0.8, color='b')[0]   

#ax.set_xticks(indices)
#ax.set_ylabel('Calibrated Distance (mm)',fontsize=14)
#ax.set_xlabel('Sensor Index',fontsize=14)
#fig.suptitle('Gripper IR Visualisation', fontsize=20)
fig.canvas.set_window_title('Linear Camera View') 
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_aspect('equal')
ax.hold(True)
background = fig.canvas.copy_from_bbox(ax.bbox) 

fig.canvas.draw()
fig.show()

#fig.canvas.mpl_connect('close_event', handle_close)
raw_input("Press enter to continue")

def updatePlot():
    global plt_data, plt_direction, plt_data_peak00, plt_data_peak10, plt_data_peak11, plt_data_peak01
    x = np.cos(angles - angle_off) * radii_scaled
    y = np.sin(angles - angle_off) * radii_scaled
     
    plt_data.set_data(x, y)
    plt_direction.remove()
    plt_direction = ax.arrow(0, 0, radius*np.cos(compass), radius*np.sin(compass), width=0.03)

    plt_data_peak00.set_data([0, np.cos(angles[peak00] + angle_off) * radius * 2], [0, np.sin(angles[peak00] + angle_off) * radius * 2])
    plt_data_peak10.set_data([0, np.cos(angles[peak10] + angle_off) * radius * 2], [0, np.sin(angles[peak10] + angle_off) * radius * 2])
    plt_data_peak11.set_data([0, np.cos(angles[peak11] + angle_off) * radius * 2], [0, np.sin(angles[peak11] + angle_off) * radius * 2])
    plt_data_peak01.set_data([0, np.cos(angles[peak01] + angle_off) * radius * 2], [0, np.sin(angles[peak01] + angle_off) * radius * 2])

    fig.canvas.draw()

updatePlot()

#raw_input("Press enter to continue")


#-------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------#

# unix python keyboard listening magic
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    
def advanced_keypress_detection_on():
    termios.tcsetattr(fd, termios.TCSANOW, newattr)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

def advanced_keypress_detection_off():
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

class keyboard_command_thread(threading.Thread):
    '''basic keypress command functionality'''
    
    def __init__ (self):
        threading.Thread.__init__(self)
        self.listening = True

    def run(self):
        advanced_keypress_detection_on()
        print '\nPress h for a list of keyboard commands'
        while True:
            try:
                if(self.listening):
                    k = sys.stdin.read(1)
                    command = -1
                    if(k=='h'):
                        self.help()
                    elif(k=='q'):
                        self.quit()

            except IOError: pass
                
    def help(self):
        print '\nCommands: h = help\n\tq = quit\n'

    def quit(self):
        global serialObj
        advanced_keypress_detection_off()
        if serialObj:
            serialObj.close()
        sys.exit()

if __name__ == '__main__':
    global portname

    # grab port from command line
    if(len(sys.argv) < 3):
        print "\nUsage: \n\tvisualize.py -p portname\n"
        sys.exit()

    if(sys.argv[1] == "-p"):
        port= sys.argv[2]

    print 'trying to connect to port ' + port

    # setup keyboard command thread
    kct = keyboard_command_thread()

    if not kct.isAlive():
        kct.start()

    # this is our exit strategy, ensuring that the process is killed upon exit
    def handle_close(evt):
        sys.exit(0)   

    # open the visualization window
    fig.canvas.draw()
    fig.show()
    #fig.canvas.mpl_connect('close_event', handle_close)

    #sys.exit(0)
    serialObj = serial.Serial(port, speed, timeout=0.5)
    display = 0
    if not serialObj:
        print 'could not open port'
        sys.exit(1)

    while 1:
        #readSensors()
        buf = serialObj.read()
        #print len(buf)
        if len(buf) < 1 or not buf.islower():
            time.sleep(10/1000)
            continue
        
        sensor_id = buf

        if sensor_id == 'a':
            print 'received a'
            tmp = serialObj.read(num_px)
            for b in range(0,612):
                data[b] = ord(tmp[b])
                radii_scaled = radius + data * scale * radius
            #    print b, ord(tmp[b])
            #updatePlot()

        elif sensor_id == 'b':
            # expecting a string of 3 bytes (leading zeros needed!)
            tmp = float(serialObj.read(3))
            print 'received b', tmp
            if tmp >= 0 and tmp <= 360:
                compass = np.radians(tmp)
                updatePlot()

        elif sensor_id == 'c':
            # expecting a string of 3 bytes (leading zeros needed!)
            tmp = float(serialObj.read(3))
            print 'received c', tmp
            peak00 = tmp
            #updatePlot()

        elif sensor_id == 'd':
            # expecting a string of 3 bytes (leading zeros needed!)
            tmp = float(serialObj.read(3))
            print 'received d', tmp
            peak10 = tmp
            #updatePlot()

        elif sensor_id == 'e':
            # expecting a string of 3 bytes (leading zeros needed!)
            tmp = float(serialObj.read(3))
            print 'received e', tmp
            peak11 = tmp
            #updatePlot()

        elif sensor_id == 'f':
            # expecting a string of 3 bytes (leading zeros needed!)
            tmp = float(serialObj.read(3))
            print 'received f', tmp
            peak01 = tmp
            #updatePlot()

 
        

        # display every once in a while
        #display += 1
        #if display == 10:
        #    display = 0
        #    # redraw
        #    fig.canvas.restore_region(background)
        #    fig.canvas.blit(ax.bbox)
        #    fig.canvas.draw()
#            out = str(chr(randint(0,26) + ord('A'))) + str(randint(0,140))
#            serialObj.write(out)

