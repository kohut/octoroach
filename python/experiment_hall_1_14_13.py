#!/usr/bin/env python
"""
authors: stanbaek, apullin

"""
from lib import command
import time,sys
import serial
import shared

from or_helpers_kohut import *


###### Operation Flags ####
SAVE_DATA1 = True 
RESET_R1 = True  

EXIT_WAIT   = False

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Robot('\x20\x52', xb)
    
    shared.ROBOTS = [R1] #This is neccesary so callbackfunc can reference robots
    shared.xb = xb           #This is neccesary so callbackfunc can halt before exit
    
    #if SAVE_DATA1:
    #    shared.dataFileName = findFileName();
    #    print "Data file:  ", shared.dataFileName

    if RESET_R1:
        R1.reset()
        time.sleep(0.35)
    
    # Query
    R1.query( retries = 8 )
    
    #Verify all robots can be queried
    verifyAllQueried()  #exits on failure

    
    # motorgains = [200,2,0,2,0,    200,2,0,2,0]
    # [Kp Ki Kd Kanti-wind ff]
    # now uses back emf velocity as d term
    motorgains = [400,0,400,0,200, 400,0,200,0,200]
    throttle = [0,0]
    #duration = 10*(32*16 - 1) # 21.3 gear ratio, 2 counts/rev
    cycle_length = 248 #ms
    duration = 20*cycle_length # num of cycles you want to run
    # velocity profile
    # [time intervals for setpoints]
    # [position increments at set points]
    # [velocity increments]   muliplied by 256
    #intervals = [0xe0, 0x70, 0xe0, 0x70]  # total 672 ms, = 16 ms per half rev
    delta = [8,8,8,8]  # adds up to 32 counts (16*2)
    #intervals = [0x11e, 0x32, 0x32, 0x11e]  # total 672 ms, = 16 ms per half rev
    #vel = [9,51,51,9]
    intervals = [cycle_length/4, cycle_length/4, cycle_length/4, cycle_length/4]  # need to figure out rounding...
    #intervals = [100, 100, 100, 100]
    #vel = [17,112,112,17]
    vel = [0, 0, 0, 0]
    
    R1.setHallGains(motorgains, retries = 8)
    #Verify all robots have motor gains set
    verifyAllHallGainsSet()   #exits on failure
    
    params = hallParams(motorgains, throttle, duration, delta, intervals, vel)
    
    R1.setVelProfile(params)
    
    tailGains = [8000,20,1000,0,0] #tuned 7/12/12
    #tailGains = [8000,0,0,0,0] # try just P for rotary control
    R1.setTailGains(tailGains)
    #Verify all robots have tail gains set
    verifyAllTailGainsSet()  #exits on failure

    #### Do not send more than 5 move segments per packet!   ####
    #### Instead, send multiple packets, and don't use       ####
    ####    calcNumSamples() below, manually calc numSamples ####
    #### This will be fixed to be automatic in the future.   ####

    #Move segment format:
    # [value1, value2 , segtime , move_seg_type , param1 , param2, param3]
    # - value1 , value2 are initial setpoints for leg speed for each segment
    # - segtime is the length of the segment, in milliseconds
    # - move_seg_type is one of the types enumerated at the top of or_helpers.py
    # MOVE_SEG_CONSTANT: param1 , param2 , param3 have no effect, set to 0.
    # MOVE_SEG_RAMP    : param1 and param2 are left/right ramp rates, in legs speed per
    #       second. param3 has no effect.
    # MOVE_SEG_SIN     : Not implemented.
    # MOVE_SEG_TRI     : Not implemented.
    # MOVE_SEG_SAW     : Not implemented.
    # MOVE_SEG_IDLE    : value1,value2 and params have no efffect. Disables leg speed controller.
    # MOVE_SEG_LOOP_DECL: Turns on move queue looping. value1,value2, and params have no effect.
    # MOVE_SEG_LOOP_CLEAR: Turns off move queue looping.value1,value2, and params have no effect.
    # MOVE_SEG_QFLUSH  : Flushes all following items in move queue. value1,value2, and params have no effect.

    #Constant example
    #moves = 1
    #moveq = [moves, \
    #         135, 135, 10000,   MOVE_SEG_CONSTANT, 0, 0, 0]
             
    #Ramp example
    
    
    init_moves = 2
    numMoves = 3
    init_time = 1000
    exp_time = 1000
    throttle = 300
    start_angle = 60.0

    #tailq_init = [init_moves, \
    #        0.0, init_time, TAIL_SEG_CONSTANT, 0, 0, 0,
     #       0.0, init_time, TAIL_SEG_CONSTANT, start_angle, 0, 0
     #       ]
    

    moveq1 = [numMoves, \
        throttle, throttle, 1.0*exp_time,   MOVE_SEG_CONSTANT, 0,  0,  0, STEER_MODE_INCREASE, int(round(shared.deg2count*0.0)),
        throttle, throttle, exp_time,   MOVE_SEG_CONSTANT, 0,  0,  0, STEER_MODE_OFF, int(round(shared.deg2count*0.0)),
        throttle, throttle, 1.0*exp_time,   MOVE_SEG_CONSTANT, 0,  0,  0, STEER_MODE_INCREASE, int(round(shared.deg2count*0.0))
        #2*throttle, 2*throttle, exp_time/2,   MOVE_SEG_CONSTANT, 0,  0,  0, STEER_MODE_OFF, int(round(shared.deg2count*0.0)),
        #throttle, throttle, exp_time,   MOVE_SEG_CONSTANT, 0,  0,  0, STEER_MODE_SPLIT, int(round(shared.deg2count*0.0))
        ]
    
    tailq = [numMoves, \
            0.0, 1.0*exp_time, TAIL_SEG_CONSTANT, 0, 0, 0,
            30.0, exp_time, TAIL_SEG_CONSTANT, 0, 0, 0,
            0.0, 1.0*exp_time, TAIL_SEG_CONSTANT, 0, 0, 0
            #0.0, exp_time/2, TAIL_GYRO_CONTROL, 90, 0, 0,
            #0.0, exp_time, TAIL_GYRO_CONTROL, 0, 0, 0
            ]
        
    #Timing settings
    R1.leadinTime = 500;
    R1.leadoutTime = 500;
    R1.setupImudata(moveq1) 
    
    #Flash must be erased to save new data
    if SAVE_DATA1:
        #This needs to be done to prepare the .imudata variables in each robot object
        
        R1.eraseFlashMem()

    # Pause and wait to start run, including leadin time
    print ""
    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print params
    R1.proceed(params)
    print ""
    
    # Trigger telemetry save, which starts as soon as it is received
    
    #### Make when saving anything, this if is set ####
    #### to the proper "SAVE_DATA"                 ####
    
    ####### Initialize tail before experiment
    #R1.sendTailQueue(tailq_init)
    #time.sleep(2.0)
    ##################
    
    
    if SAVE_DATA1:
        R1.startTelemetrySave()

    time.sleep(R1.leadinTime / 1000.0)
    #Send the move queue to the robot; robot will start processing it
    #as soon as it is received
    #R1.sendMoveQueue(moveq1)
    R1.sendTailQueue(tailq)
    
    maxtime = 0
    for r in shared.ROBOTS:
        tottime =  r.runtime + r.leadoutTime
        if tottime > maxtime:
            maxtime = tottime
    
    #Wait for robots to do runs
    time.sleep(maxtime / 1000.0)
    
    
    
    if SAVE_DATA1:
        raw_input("Press Enter to start telemtry readback ...")
        R1.downloadTelemetry()

    if EXIT_WAIT:  #Pause for a Ctrl + Cif specified
        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                break

    print "Done"
    xb_safe_exit()


#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        shared.xb.halt()
        shared.ser.close()
    #except Exception as args:
    #    print "\nGeneral exception:",args
    #    print "Attemping to exit cleanly..."
    #    shared.xb.halt()
    #    shared.ser.close()
    #    sys.exit()
