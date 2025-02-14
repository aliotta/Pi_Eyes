#!/usr/bin/python

# This is a hasty port of the Teensy eyes code to Python...all kludgey with
# an embarrassing number of globals in the frame() function and stuff.
# Needed to get SOMETHING working, can focus on improvements next.

import Adafruit_ADS1x15
import argparse
import math
import pi3d
import random
import thread
import time
import RPi.GPIO as GPIO
from svg.path import Path, parse_path
from xml.dom.minidom import parse
from gfxutil import *

# INPUT CONFIG for eye motion ----------------------------------------------
# ANALOG INPUTS REQUIRE SNAKE EYES BONNET

JOYSTICK_X_IN   = 1    # Analog input for eye horiz pos (-1 = auto)
JOYSTICK_Y_IN   = 2    # Analog input for eye vert position (")
JOYSTICK_Z_IN   = 0
PUPIL_IN        = 0    # Analog input for pupil control (-1 = auto)
JOYSTICK_X_FLIP = False # If True, reverse stick X axis
JOYSTICK_Y_FLIP = False # If True, reverse stick Y axis
PUPIL_IN_FLIP   = False # If True, reverse reading from PUPIL_IN
TRACKING        = True  # If True, eyelid tracks pupil
PUPIL_SMOOTH    = 16    # If > 0, filter input from PUPIL_IN
PUPIL_MIN       = 0.0   # Lower analog range from PUPIL_IN
PUPIL_MAX       = 1.0   # Upper "


# GPIO initialization ------------------------------------------------------

GPIO.setmode(GPIO.BCM)


# ADC stuff ----------------------------------------------------------------

if JOYSTICK_X_IN >= 0 or JOYSTICK_Y_IN >= 0 or PUPIL_IN >= 0:
	adc      = Adafruit_ADS1x15.ADS1015()
	adcValue = [0] * 4
else:
	adc = None

# Because ADC reads are blocking operations, they normally would slow down
# the animation loop noticably, especially when reading multiple channels
# (even when using high data rate settings).  To avoid this, ADC channels
# are read in a separate thread and stored in the global list adcValue[],
# which the animation loop can read at its leisure (with immediate results,
# no slowdown).
def adcThread(adc, dest):
	while True:
		for i in range(len(dest)):
			# ADC input range is +- 4.096V
			# ADC output is -2048 to +2047
			# Analog inputs will be 0 to ~3.3V,
			# thus 0 to 1649-ish.  Read & clip:
			n = adc.read_adc(i, gain=1)
			if   n <    0: n =    0
			elif n > 1649: n = 1649
			dest[i] = n / 1649.0 # Store as 0.0 to 1.0
		time.sleep(0.01) # 100-ish Hz

# Start ADC sampling thread if needed:
if adc:
	thread.start_new_thread(adcThread, (adc, adcValue))


# Load SVG file, extract paths & convert to point lists --------------------

dom               = parse("graphics/eye_lidless.svg")
vb                = getViewBox(dom)
pupilMinPts       = getPoints(dom, "pupilMin"      , 32, True , True )
pupilMaxPts       = getPoints(dom, "pupilMax"      , 32, True , True )
irisPts           = getPoints(dom, "iris"          , 32, True , True )
scleraFrontPts    = getPoints(dom, "scleraFront"   ,  0, False, False)
scleraBackPts     = getPoints(dom, "scleraBack"    ,  0, False, False)

domBackOfEye               = parse("graphics/eye_back.svg")
vbBackOfEye                = getViewBox(domBackOfEye)
scleraBackPtsBackOfEye     = getPoints(domBackOfEye, "scleraBack"    ,  0, False, False)


# Set up display and initialize pi3d ---------------------------------------

DISPLAY = pi3d.Display.create(samples=4)
DISPLAY.set_background(0, 0, 0, 1) # r,g,b,alpha

# eyeRadius is the size, in pixels, at which the whole eye will be rendered
# onscreen.  eyePosition, also pixels, is the offset (left or right) from
# the center point of the screen to the center of each eye.  This geometry
# is explained more in-depth in fbx2.c.
eyePosition = DISPLAY.width / 4
eyeRadius   = 128  # Default; use 240 for IPS screens

parser = argparse.ArgumentParser()
parser.add_argument("--radius", type=int)
args = parser.parse_args()
if args.radius:
	eyeRadius = args.radius


# A 2D camera is used, mostly to allow for pixel-accurate eye placement,
# but also because perspective isn't really helpful or needed here, and
# also this allows eyelids to be handled somewhat easily as 2D planes.
# Line of sight is down Z axis, allowing conventional X/Y cartesion
# coords for 2D positions.
cam    = pi3d.Camera(is_3d=False, at=(0,0,0), eye=(0,0,-1000))
shader = pi3d.Shader("uv_light")
light  = pi3d.Light(lightpos=(0, -500, -500), lightamb=(.2, .2, .2))


# Load texture maps --------------------------------------------------------
irisMap   = pi3d.Texture("graphics/iris.jpg"  , mipmap=False,
              filter=pi3d.GL_LINEAR)
scleraMap = pi3d.Texture("graphics/sclera.png", mipmap=False,
              filter=pi3d.GL_LINEAR, blend=True)
scleraMapBackOfEye = pi3d.Texture("graphics/sclera_back.png", mipmap=False,
              filter=pi3d.GL_LINEAR, blend=True)

# U/V map may be useful for debugging texture placement; not normally used
#uvMap     = pi3d.Texture("graphics/uv.png"    , mipmap=False,
#              filter=pi3d.GL_LINEAR, blend=False, m_repeat=True)


# Initialize static geometry -----------------------------------------------

# Transform point lists to eye dimensions
scalePoints(pupilMinPts      , vb, eyeRadius)
scalePoints(pupilMaxPts      , vb, eyeRadius)
scalePoints(irisPts          , vb, eyeRadius)
scalePoints(scleraFrontPts   , vb, eyeRadius)
scalePoints(scleraBackPts    , vb, eyeRadius)
scalePoints(scleraBackPtsBackOfEye    , vbBackOfEye, eyeRadius)

irisRegenThreshold = 0.0
a = pointsBounds(pupilMinPts) # Bounds of pupil at min size (in pixels)
b = pointsBounds(pupilMaxPts) # " at max size
maxDist = max(abs(a[0] - b[0]), abs(a[1] - b[1]), # Determine distance of max
              abs(a[2] - b[2]), abs(a[3] - b[3])) # variance around each edge
# maxDist is motion range in pixels as pupil scales between 0.0 and 1.0.
# 1.0 / maxDist is one pixel's worth of scale range.  Need 1/4 that...
if maxDist > 0: irisRegenThreshold = 0.25 / maxDist

# Generate initial iris meshes; vertex elements will get replaced on
# a per-frame basis in the main loop, this just sets up textures, etc.
rightIris = meshInit(32, 4, True, 0, 0.5/irisMap.iy, False)
rightIris.set_textures([irisMap])
rightIris.set_shader(shader)
# Left iris map U value is offset by 0.5; effectively a 180 degree
# rotation, so it's less obvious that the same texture is in use on both.
leftIris = meshInit(32, 4, True, 0.5, 0.5/irisMap.iy, False)
leftIris.set_textures([irisMap])
leftIris.set_shader(shader)
irisZ = zangle(irisPts, eyeRadius)[0] * 0.99 # Get iris Z depth, for later

# Generate scleras for each eye...start with a 2D shape for lathing...
angle1 = zangle(scleraFrontPts, eyeRadius)[1] # Sclera front angle
angle2 = zangle(scleraBackPts , eyeRadius)[1] # " back angle
aRange = 180 - angle1 - angle2
pts    = []
for i in range(24):
	ca, sa = pi3d.Utility.from_polar((90 - angle1) - aRange * i / 23)
	pts.append((ca * eyeRadius, sa * eyeRadius))

#angle1BackOfEye = zangle(scleraFrontPtsBackOfEye, eyeRadius)[1] # Sclera front angle
angle2BackOfEye = zangle(scleraBackPtsBackOfEye , eyeRadius)[1] # " back angle
aRangeBackOfEye = 180 - angle2BackOfEye
ptsBackOfEye    = []
for i in range(24):
	caBackOfEye, saBackOfEye = pi3d.Utility.from_polar((90 + 10) - aRangeBackOfEye * i / 23)
	ptsBackOfEye.append((caBackOfEye * eyeRadius, saBackOfEye * eyeRadius))

# Scleras are generated independently (object isn't re-used) so each
# may have a different image map (heterochromia, corneal scar, or the
# same image map can be offset on one so the repetition isn't obvious).
leftEye = pi3d.Lathe(path=pts, sides=64)
leftEye.set_textures([scleraMap])
leftEye.set_shader(shader)
reAxis(leftEye, 0)
rightEye = pi3d.Lathe(path=pts, sides=64)
rightEye.set_textures([scleraMap])
rightEye.set_shader(shader)
reAxis(rightEye, 0.5) # Image map offset = 180 degree rotation

leftEyeBackOfEye = pi3d.Lathe(path=ptsBackOfEye, sides=64)
leftEyeBackOfEye.set_textures([scleraMapBackOfEye])
leftEyeBackOfEye.set_shader(shader)
reAxis(leftEyeBackOfEye, 0)
rightEyeBackOfEye = pi3d.Lathe(path=ptsBackOfEye, sides=64)
rightEyeBackOfEye.set_textures([scleraMapBackOfEye])
rightEyeBackOfEye.set_shader(shader)
reAxis(rightEyeBackOfEye, 0.5) # Image map offset = 180 degree rotation


# Init global stuff --------------------------------------------------------

mykeys = pi3d.Keyboard() # For capturing key presses

startX       = random.uniform(-30.0, 30.0)
n            = math.sqrt(900.0 - startX * startX)
startY       = random.uniform(-n, n)
destX        = startX
destY        = startY
curX         = startX
curY         = startY
moveDuration = random.uniform(0.075, 0.175)
holdDuration = random.uniform(0.1, 1.1)
startTime    = 0.0
isMoving     = False

startXR      = random.uniform(-30.0, 30.0)
n            = math.sqrt(900.0 - startX * startX)
startYR      = random.uniform(-n, n)
destXR       = startXR
destYR       = startYR
curXR        = startXR
curYR        = startYR
moveDurationR = random.uniform(0.075, 0.175)
holdDurationR = random.uniform(0.1, 1.1)
startTimeR    = 0.0
isMovingR     = False

frames        = 0
frameCount    = 0
beginningTime = time.time()

rightEye.positionX(-eyePosition)
rightIris.positionX(-eyePosition)
leftEye.positionX(eyePosition)
leftIris.positionX(eyePosition)

rightEyeBackOfEye.positionX(-eyePosition)
leftEyeBackOfEye.positionX(eyePosition)

trackingPos = 0.3
trackingPosR = 0.3
currentPupilScale       =  0.5
prevPupilScale          = -1.0 # Force regen on first frame

showIrisAndPupil = True
pressed = False
# Generate one frame of imagery
def frame(p):

	global startX, startY, destX, destY, curX, curY
	global startXR, startYR, destXR, destYR, curXR, curYR
	global moveDuration, holdDuration, startTime, isMoving
	global moveDurationR, holdDurationR, startTimeR, isMovingR
	global frames
	global frameCount
	global leftIris, rightIris
	global pupilMinPts, pupilMaxPts, irisPts, irisZ
	global prevPupilScale
	global irisRegenThreshold
	global leftEye, rightEye
	global trackingPos
	global trackingPosR
	global showIrisAndPupil
	global pressed

	DISPLAY.loop_running()

	now = time.time()
	dt  = now - startTime
	dtR  = now - startTimeR

	frames += 1
	if adcValue[JOYSTICK_Z_IN] == 0.0:
	  frameCount += 1
	  if frameCount >= 15 and not pressed:
	    showIrisAndPupil = not showIrisAndPupil
	    pressed = True
	else:
	  frameCount = 0
	  pressed = False
#	if(now > beginningTime):
#		print(frames/(now-beginningTime))

	# Eye position from analog inputs
	curX = adcValue[JOYSTICK_X_IN]
	curY = adcValue[JOYSTICK_Y_IN]
	if JOYSTICK_X_FLIP: curX = 1.0 - curX
	if JOYSTICK_Y_FLIP: curY = 1.0 - curY
	curX = -30.0 + curX * 60.0
	curY = -30.0 + curY * 60.0


	convergence = 2.0
	if not showIrisAndPupil:
	  # Regenerate iris geometry only if size changed by >= 1/4 pixel
	  if abs(p - prevPupilScale) >= irisRegenThreshold:
	    # Interpolate points between min and max pupil sizes
	    interPupil = pointsInterp(pupilMinPts, pupilMaxPts, p)
	    # Generate mesh between interpolated pupil and iris bounds
	    mesh = pointsMesh(None, interPupil, irisPts, 4, -irisZ, True)
	    # Assign to both eyes
	    leftIris.re_init(pts=mesh)
	    rightIris.re_init(pts=mesh)
	    prevPupilScale = p
          rightIris.rotateToX(curY)
	  rightIris.rotateToY(curX - convergence)
	  rightIris.draw() 
	  rightEye.rotateToX(curY)
	  rightEye.rotateToY(curX - convergence)
	  rightEye.draw()

	  leftIris.rotateToX(curY)
	  leftIris.rotateToY(curX + convergence)
	  leftIris.draw()
	  leftEye.rotateToX(curY)
	  leftEye.rotateToY(curX + convergence)
	  leftEye.draw()
	else:
	  rightEyeBackOfEye.rotateToX(curY)
	  rightEyeBackOfEye.rotateToY(curX - convergence)
	  rightEyeBackOfEye.draw()

	  leftEyeBackOfEye.rotateToX(curY)
	  leftEyeBackOfEye.rotateToY(curX + convergence)
	  leftEyeBackOfEye.draw()

	k = mykeys.read()
	if k==27:
		mykeys.close()
		DISPLAY.stop()
		exit(0)

# MAIN LOOP -- runs continuously -------------------------------------------

while True:
	frame(0.5)
