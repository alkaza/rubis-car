#!/usr/bin/env python

import rospy
from race.msg import drive_param
import curses
import sys
import signal

def signal_handler(signal, frame):
    sys.exit('KeyboardInterrupt: Terminating.. ')


forward = 0;
left = 0;

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_talker', anonymous=True)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)

stdscr.refresh()

key = ''
signal.signal(signal.SIGINT, signal_handler)


# keys do not show up in the terminal after Ctrl+C
# If I don't add try & except KeyboardInterrupt
print('** keyboard.py **')
try:
    while key != ord('q'):
        key = stdscr.getch()
	stdscr.refresh()
        
        if key == curses.KEY_UP: 
            forward = forward + 1;
            stdscr.addstr(2, 20, "Up  ")
            stdscr.addstr(2, 25, '%.2f' % forward)
            stdscr.addstr(5, 20, "    ")
        elif key == curses.KEY_DOWN:
            forward = forward - 1; 
            stdscr.addstr(2, 20, "Down ")
            stdscr.addstr(2, 25, '%.2f' % forward)
            stdscr.addstr(5, 20, "    ")
        if key == curses.KEY_RIGHT:
            left = left - 1; 
            stdscr.addstr(3, 20, "Right ")
            stdscr.addstr(3, 25, '%.2f' % left)
            stdscr.addstr(5, 20, "    ")
        elif key == curses.KEY_LEFT:
            left = left + 1; 
            stdscr.addstr(3, 20, "Left ")
            stdscr.addstr(3, 25, '%.2f' % left)
            stdscr.addstr(5, 20, "    ")
        if key == curses.KEY_DC:
            left = 0
            forward = 0
            stdscr.addstr(5, 20, "Stop")
        msg = drive_param()
        msg.velocity = forward
        msg.angle = left
        pub.publish(msg)
except KeyboardInterrupt:
    print("??")
    #curses.nocbreak()
    #stdscr.keypad(False)
    #curses.echo()
    #curses.endwin()
    #sys.exit(-1) 
finally:
    pass
    curses.endwin()
