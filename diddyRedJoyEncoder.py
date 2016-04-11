#!/usr/bin/env python
# coding: Latin-1

# Load library functions we want
import time
import os
import sys
import pygame
import PicoBorgRev
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--fixed_power', type=float, default=.70)
parser.add_argument('--useFixedPower', type=int, default=1)
args = parser.parse_args()

# Re-direct our output to standard error, we need to ignore standard out to hide some nasty print statements from pygame
stdout = sys.stdout
sys.stdout = sys.stderr

# Setup the PicoBorg Reverse
PBR = PicoBorgRev.PicoBorgRev()
#PBR.i2cAddress = 0x44                  # Uncomment and change the value if you have changed the board address
PBR.Init()
if not PBR.foundChip:
    boards = PicoBorgRev.ScanForPicoBorgReverse()
    if len(boards) == 0:
        print 'No PicoBorg Reverse found, check you are attached :)'
    else:
        print 'No PicoBorg Reverse at address %02X, but we did find boards:' % (PBR.i2cAddress)
        for board in boards:
            print '    %02X (%d)' % (board, board)
        print 'If you need to change the I²C address change the setup line so it is correct, e.g.'
        print 'PBR.i2cAddress = 0x%02X' % (boards[0])
    sys.exit()
#PBR.SetEpoIgnore(True)                 # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
# Ensure the communications failsafe has been enabled!
failsafe = False
for i in range(5):
    PBR.SetCommsFailsafe(True)
    failsafe = PBR.GetCommsFailsafe()
    if failsafe:
        break
if not failsafe:
    print 'Board %02X failed to report in failsafe mode!' % (PBR.i2cAddress)
    sys.exit()
PBR.ResetEpo()

# Settings for the joystick
axisUpDown = 1                          # Joystick axis to read for up / down position
axisUpDownInverted = False              # Set this to True if up and down appear to be swapped
axisLeftRight = 2                       # Joystick axis to read for left / right position
axisLeftRightInverted = False           # Set this to True if left and right appear to be swapped
buttonResetEpo = 3                      # Joystick button number to perform an EPO reset (Start)
buttonSlow = 8                          # Joystick button number for driving slowly whilst held (L2)
slowFactor = 0.5                        # Speed to slow to when the drive slowly button is held, e.g. 0.5 would be half speed
buttonFastTurn = 9                      # Joystick button number for turning fast (R2)
interval = 0.05                         # Time between updates in seconds, smaller responds faster but uses more processor time

# Power settings
voltageIn = 12.0                        # Total battery voltage to the PicoBorg Reverse
voltageOut = 12.0 * 0.95                # Maximum motor voltage, we limit it to 95% to allow the RPi to get uninterrupted power

# Setup the power limits
if voltageOut > voltageIn:
    maxPower = 1.0
else:
    maxPower = voltageOut / float(voltageIn)

def get_velocity (power):
  if power < 0: return -0.3937
  return 0.3937

# Setup pygame and wait for the joystick to become available
PBR.MotorsOff()
os.environ["SDL_VIDEODRIVER"] = "dummy" # Removes the need to have a GUI window
pygame.init()
#pygame.display.set_mode((1,1))
print 'Waiting for joystick... (press CTRL+C to abort)'
while True:
    try:
        try:
            pygame.joystick.init()
            # Attempt to setup the joystick
            if pygame.joystick.get_count() < 1:
                # No joystick attached, toggle the LED
                PBR.SetLed(not PBR.GetLed())
                pygame.joystick.quit()
                time.sleep(0.5)
            else:
                # We have a joystick, attempt to initialise it!
                joystick = pygame.joystick.Joystick(0)
                break
        except pygame.error:
            # Failed to connect to the joystick, toggle the LED
            PBR.SetLed(not PBR.GetLed())
            pygame.joystick.quit()
            time.sleep(0.5)
    except KeyboardInterrupt:
        # CTRL+C exit, give up
        print '\nUser aborted'
        PBR.SetLed(True)
        sys.exit()
print 'Joystick found'
joystick.init()
PBR.SetLed(False)

# Setting up feedback (encoder) mode
PBR.SetEncoderMoveMode(True)         # Enable / disable feedback following mode
PBR.GetEncoderMoveMode()                # Check if we are in feedback following mode


try:
    print 'Press CTRL+C to quit'
    driveLeft = 0.0
    driveRight = 0.0
    running = True
    hadEvent = False
    upDown = 0.0
    leftRight = 0.0
    # Loop indefinitely

    # Fixed power (for now)
    fixed_power = args.fixed_power
    useFixedPower = args.useFixedPower



    t1 = time.time()
    nomotion_count = 0
    MOTION_THRESH = 100
    while running:
        # Get the latest events from the system
        hadEvent = False
        events = pygame.event.get()
        turn = "straight"
        # Handle each event individually
        for event in events:
            if event.type == pygame.QUIT:
                # User exit
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                # A button on the joystick just got pushed down
                hadEvent = True                    
            elif event.type == pygame.JOYAXISMOTION:
                # A joystick has been moved
                hadEvent = True
            if hadEvent:
                # Read axis positions (-1 to +1)
                if axisUpDownInverted:
                    upDown = -joystick.get_axis(axisUpDown)
                else:
                    upDown = joystick.get_axis(axisUpDown)
                if axisLeftRightInverted:
                    leftRight = -joystick.get_axis(axisLeftRight)
                else:
                    leftRight = joystick.get_axis(axisLeftRight)
                # Apply steering speeds
                if not joystick.get_button(buttonFastTurn):
                    leftRight *= 0.5
                # Determine the drive power levels
                driveLeft = -upDown
                driveRight = -upDown
                
                # Overwrite with fixed power
                if useFixedPower==1:
                    if upDown < 0:
                        driveLeft  = fixed_power/.95
                        driveRight = fixed_power/.95
                    elif upDown>0:
                        driveLeft  = -fixed_power/.95
                        driveRight = -fixed_power/.95


               
                if leftRight < -0.05:
                    # Turning left
                    driveRight = driveRight
                    driveLeft = -driveLeft
                elif leftRight > 0.05:
                    # Turning right
                    driveLeft = driveLeft
                    driveRight = -driveRight
                # Check for button presses
                if joystick.get_button(buttonResetEpo):
                    PBR.ResetEpo()
                if joystick.get_button(buttonSlow):
                    driveLeft *= slowFactor
                    driveRight *= slowFactor
                # Set the motors to the new speeds
                
                
                PBR.SetMotor1(driveRight * maxPower)
                PBR.SetMotor2(-driveLeft * maxPower)
                    
                #while True:
                #    PBR.SetMotor1(.95)
                #    PBR.SetMotor2(.95)

                #t2 = time.time()
                #dt = t2-t1
                #print dt
                #t1 = t2

                # Set Encoder Power level
                #PBR.SetEncoderSpeed(driveRight * maxPower)              # Set the speed of both motors in feedback following mode
                #print driveRight*maxPower
                
                # Move motors forward
                #PBR.EncoderMoveMotor1(1)           # Move motor 1 a number of counts in a direction
                #PBR.EncoderMoveMotor2(-1)           # Move motor 2 a number of counts in a direction
                #PBR.WaitWhileEncoderMoving(.000250)
                
        # Change the LED to reflect the status of the EPO latch
        PBR.SetLed(PBR.GetEpo())
        # Wait for the interval period
        time.sleep(interval)
       	m1 = PBR.GetMotor1()
	m2 = PBR.GetMotor2()
	if m1 == m2: stdout.write('0\n')
	else:
	  direction = 1
	  if m2 > m1: direction = -1
 	  vel = get_velocity(direction)
	  stdout.write('{}\n'.format(vel))
	 #string = '{} {}\n'.format(PBR.GetMotor1(), PBR.GetMotor2())
        #print string
        #if hadEvent: 
        #    #stdout.write('{}\r\n'.format(velocity))
        #    nomotion_count = 0
        #else:
        #    nomotion_count += 1
        #    if nomotion_count > MOTION_THRESH:
        #        stdout.write('0\r\n')
        #        nomotion_count = 0
    # Disable all drives
    PBR.MotorsOff()
except KeyboardInterrupt:
    # CTRL+C exit, disable all drives
    PBR.MotorsOff()
print
