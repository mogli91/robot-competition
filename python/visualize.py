#!/usr/bin/python
import sys, os
import time
import serial
import numpy as np
from matplotlib import cm
from matplotlib.pylab import subplots,close
from random import randint

# for key board interrupts
import threading
import termios, fcntl

# serial port to connect to
port = '/dev/ttyACM1'
speed = 115200
serialObj = None

# sensor indices, values and names (a - z)
num_sensors = 26
reading_size = 5
sensor_idx = range(0,num_sensors)
sensor_val = np.zeros(num_sensors)
idx_map = dict()
for i in range(0, num_sensors):
    idx_map[chr(i + ord('a'))] = i

# setup animated plot
fig,ax = subplots(1)
#plt_raw = ax.plot(sensor_idx,D[0,:],'o')[0]
plt_raw_rect= ax.bar(sensor_idx, sensor_val, 1.0, color='r', alpha=0.8, align='center')
#plt_detection = ax.plot([],[],linewidth=10.0,alpha=0.8, color='y')[0]   
ax.set_xticks(sensor_idx)
ax.set_ylabel('Distance (cm)',fontsize=14)
ax.set_xlabel('Sensor ID',fontsize=14)
fig.canvas.set_window_title('Sensor Visualization') 
ax.set_xlim(-0.5, num_sensors + 0.5)
ax.set_ylim(0,140)
ax.hold(True)
background = fig.canvas.copy_from_bbox(ax.bbox) 

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
    fig.canvas.mpl_connect('close_event', handle_close)

#    print 'yes'
#    for i in sensor_idx:
#        sensor_val[i] = i * 5
#        plt_raw_rect[i].set_height(sensor_val[i])            
#    # redraw
#    fig.canvas.restore_region(background)
#    #ax.draw_artist(plt_raw_rect)
#    fig.canvas.blit(ax.bbox)
#    fig.canvas.draw()
#    sys.exit(0)

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
#            time.sleep(10/1000)
            continue
        
        sensor_id = buf
        # read value
        str_val = ''
        str_val = serialObj.read(3)

        #for count in range(0, reading_size - 1):
        #    buf = serialObj.read()
        #    if len(buf) < 1 or buf == '\0':
        #        break
        #    str_val += str(buf)
        
        display += 1
        # set value in local memory
        if len(str_val) > 0:
            print 'read ' + sensor_id + ' ' + str_val
            idx = idx_map[sensor_id]
            sensor_val[idx] = int(str_val)
            plt_raw_rect[idx].set_height(sensor_val[idx])

            if display == 10:
                # redraw
                fig.canvas.restore_region(background)
                fig.canvas.blit(ax.bbox)
                fig.canvas.draw()

                display = 0
#            out = str(chr(randint(0,26) + ord('A'))) + str(randint(0,140))
#            serialObj.write(out)

