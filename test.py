# Attachment color tag hex code dictionary
MISSIONS = {
    "1": 0xC6CAD1,
    "2": 0x2077A3,
    "3": 0x0D336B,
    "4": 0x260F17,
    "5": 0x827224,
    "6": 0x3D1121,
    "7": 0xDBB6C2,
    "8": 0x33262C
}

#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////

# Necessary libraries
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath as math

# Hub init
hub = PrimeHub()
hub.system.set_stop_button(Button.LEFT)
stp = StopWatch()
hub.light.on(Color.CYAN)
current = "4"

# Peripheral init
lw = Motor(Port.B, Direction.COUNTERCLOCKWISE)
rw = Motor(Port.F)
ml = Motor(Port.E)
mr = Motor(Port.A)
cs = ColorSensor(Port.D)
sw = ColorSensor(Port.C)

# Tuning variables
kp = 3
ki = 2
kd = 0
pm = 1.1

# IMU & motor init
while hub.imu.ready() == False:
    wait(50)
hub.imu.reset_heading(0)
lw.reset_angle(0)
rw.reset_angle(0)

#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////

# Color detection function (complex so I'll hav comments inside)
def prg():
    h,s,v = sw.hsv() # HSV is hue saturation value: way to identify a color, kinda like a hex code
    hn,sn,vn = h/360, s/100, v/100 # HSV is 0-360, 0-100, 0-100 but this makes it all 0-1
    i = int(hn*6) # Where does the color sit on color wheel after dividing into sectors 0-5
    f = (hn*6) - i # How far is it into that sector
    p,q,t = int(vn*(1-sn)*255), int(vn*(1-sn*f)*255), int(vn*(1-sn*(1-f))*255) # Finds rgb color going up, down, and staying the same
    v = int(vn*255) # Puts v on a 8bit scale
    r,g,b = [(v,t,p),(q,v,p),(p,v,t),(p,q,v),(t,p,v),(v,p,q)][i%6] # Finds RGB
    dprev = 1e6 # Sets dprev very high so that dprev < md will always return true first time
    res = "blah" # Variable that stores closest fit
    for lbl,hx in MISSIONS.items(): # Cycles through all the hex codes to find best fit
        mrc, mg, mb = (hx>>16)&255, (hx>>8)&255, hx&255 # Separates red, green, and blue
        d = math.sqrt((r-mrc)**2 + (g-mg)**2 + (b-mb)**2) # Finds "distance" btwn read and listed rgbs
        if d < dprev: # if this distance is less than the previous best, then:
            dprev = d # update dprev
            res = lbl # update the result
    return res # Returns closest fit

# Function to stop robot
def stop():
    lw.stop()
    rw.stop()

# Function to reset yaw
def reset():
    lw.stop()
    rw.stop()
    wait(200)
    hub.imu.reset_heading(0)

# Straight move function
def m(mm, acc = 1, maxspeed = 80, startspeed = 25, minspeed = 15):
    x = mm * 1.83640318952
    i = 0
    lw.reset_angle(0)
    rw.reset_angle(0)
    dg = (lw.angle() + rw.angle())/2
    yprev = hub.imu.heading()
    if (x > 0):
        s = startspeed
        while (dg < x/2 - 50):
            y = hub.imu.heading()
            dg = (lw.angle() + rw.angle())/2
            s += acc
            p = y * kp
            i += y * ki * 0.01
            d = hub.imu.angular_velocity(Axis.Z) * kd / 100
            corr = p + i + d
            lw.dc(max(minspeed, (min(s, maxspeed))) - corr)
            rw.dc(max(minspeed, (min(s, maxspeed))) + corr)
            wait(10)
        while (dg < x - 50):
            y = hub.imu.heading()
            dg = (lw.angle() + rw.angle())/2
            s -= acc
            p = y * kp
            i += y * ki * 0.01
            d = hub.imu.angular_velocity(Axis.Z) * kd / 100
            corr = p + i + d
            lw.dc(max(minspeed, (min(s, maxspeed))) - corr)
            rw.dc(max(minspeed, (min(s, maxspeed))) + corr)
            wait(10)
    else:
        s = -startspeed
        while (dg > x/2 + 50):
            y = hub.imu.heading()
            dg = (lw.angle() + rw.angle())/2
            s -= acc
            p = y * kp
            i += y * ki * 0.01
            d = hub.imu.angular_velocity(Axis.Z) * kd / 100
            corr = p + i + d
            lw.dc(min(-minspeed, (max(s, -maxspeed))) - corr)
            rw.dc(min(-minspeed, (max(s, -maxspeed))) + corr)
            wait(10)
        while (dg > x + 50):
            y = hub.imu.heading()
            dg = (lw.angle() + rw.angle())/2
            s += acc
            p = y * kp
            i += y * ki * 0.01
            d = hub.imu.angular_velocity(Axis.Z) * kd / 100
            corr = p + i + d
            lw.dc(min(-minspeed, (max(s, -maxspeed))) - corr)
            rw.dc(min(-minspeed, (max(s, -maxspeed))) + corr)
            wait(10)
    stp.reset()
    while ((abs(dg) > abs(x) + 10 or abs(dg) < abs(x) - 10) and stp.time() < 2000):
        y = hub.imu.heading()
        dg = (lw.angle() + rw.angle())/2
        p = y * kp
        lw.dc(min((x - dg) * pm - p, 100))
        rw.dc(min((x - dg) * pm + p, 100))

# Reverse pivot turn function
def rpt(dg):
    y = dg - hub.imu.heading()
    stp.reset()
    if (dg > 0):
        while (y > 0 and stp.time() < 3000):
            y = dg - hub.imu.heading()
            corr = min(30 + y, 77)
            rw.dc(-corr)
            lw.dc(0)
            wait(1)
    else:
        while (y < 0 and stp.time() < 3000):
            y = dg - hub.imu.heading()
            corr = min(30 - y, 77)
            rw.dc(0)
            lw.dc(-corr)
            wait(1)
    hub.imu.reset_heading(dg - hub.imu.heading())

# Pivot turn function
def pt(dg, spd = 77):
    y = dg - hub.imu.heading()
    stp.reset()
    if (dg > 0):
        while (y > 0 and stp.time() < 3000):
            y = dg - hub.imu.heading()
            corr = min(30 + y, spd)
            lw.dc(corr)
            rw.dc(0)
            wait(1)
    else:
        while (y < 0 and stp.time() < 1800):
            y = dg - hub.imu.heading()
            corr = min(30 - y, spd)
            lw.dc(0)
            rw.dc(corr)
            wait(1)
    hub.imu.reset_heading(dg - hub.imu.heading())

# Standard turn function
def t(dg):
    y = dg - hub.imu.heading()
    stp.reset()
    if (dg > 0):
        while (y > 0 and stp.time() < 3000):
            y = dg - hub.imu.heading()
            corr = min(30 + y, 77)
            lw.dc(corr / 1.4)
            rw.dc(-corr / 1.4)
            wait(1)
    else:
        while (y < 0 and stp.time() < 3000):
            y = dg - hub.imu.heading()
            corr = min(30 - y, 77)
            lw.dc(-corr / 1.5)
            rw.dc(corr / 1.5)
            wait(1)
    hub.imu.reset_heading(dg - hub.imu.heading())

# Black line allignment function
def blk(sp = 20):
    while (cs.color() != Color.BLACK):
        y = hub.imu.heading() * kp
        lw.dc(sp - y)
        rw.dc(sp + y)
        wait(1)

# Timed move function
def tmove(sec, spd) :
    stp.reset()
    while stp.time() < sec * 1000:
        lw.dc(spd - hub.imu.heading())
        rw.dc(spd + hub.imu.heading())

# Show to judges
def test():
    reset()
    m(240)
    m(-240)
    pt(90)
    t(-90)
    pt(-90)
    t(90)
    rpt(90)
    t(-90)
    rpt(-90)
    t(90)
    stop()

#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////
#///////////////////////////////////////////////////////////////////////////////

reset()

# Main loop
while True:
    clm = prg()
    if clm != current:
        hub.display.char(clm)
        current = clm
    if clm == "1" and Button.CENTER in hub.buttons.pressed():
        reset()
        mr.run_angle(3200, 300, wait=False)
        m(150)
        pt(-20)
        m(470)
        pt(65)
        mr.run_angle(3800, -300, wait=False)
        stop()
        wait(300)
        m(125)
        ml.run_angle(100000, 340)
        wait(700)
        mr.run_angle(3200, 500)
        m(-137)
        pt(-29)
        m(-160)
        pt(-32)
        m(-590)
        stop()
    elif clm == "2" and Button.CENTER in hub.buttons.pressed():
        reset()
        mr.run_angle(250, -140)
        pt(-47.5)
        stop()
        ml.run_angle(330, 410, wait=False)
        m(520)
        stop()
        mr.run_angle(250, 140, wait=False)
        ml.run_angle(450, -410)
        ml.run_angle(350, 300)
        m(-157, startspeed=80)
        mr.run_angle(250, -150, wait=False)
        m(100)
        m(-150)
        rpt(-37)
        m(-270)
        stop()
    elif clm == "3" and Button.CENTER in hub.buttons.pressed():
        reset()
        mr.run_angle(100000, -660, wait=False)
        m(457)
        stop()
        mr.run_time(-950, 5700)
        m(-150)
        rpt(-88)
        m(1680, maxspeed=100)
        pt(88)
        m(-300)
        stop()
        while Button.CENTER not in hub.buttons.pressed():
            wait(50)
        reset()
        m(520)
        tmove(0.5, 35)
        tmove(0.3, -35)
        hub.imu.reset_heading(5 + hub.imu.heading())
        m(-520, startspeed=30)
        stop()
    elif clm == "4" and Button.CENTER in hub.buttons.pressed():
        reset()
        mr.run_angle(3200, -150, wait=False)
        ml.run_angle(100000, -450, wait=False)
        m(148)
        pt(20)
        m(750, maxspeed=95)
        pt(67)
        mr.run_angle(3200, 150, wait=False)
        ml.run_angle(100000, 450, wait=False)
        m(-100)
        m(190, maxspeed=40)
        wait(100)
        mr.run_angle(3200, -160, wait=False)
        ml.run_angle(100000, -450)
        wait(500)
        m(-170)
        pt(110)
        m(700, maxspeed=100)
        stop()
    elif clm == "5" and Button.CENTER in hub.buttons.pressed():
        reset()
        m(150)
        pt(15)
        m(538)
        pt(-60)
        tmove(1, 50)
        mr.run_angle(100000, 250)
        m(-105)
        rpt(55)
        tmove(0.5, -30)
        stop()
        mr.run_angle(100000, -150)
        m(-600, maxspeed=100)
        stop()
    elif clm == "6" and Button.CENTER in hub.buttons.pressed():
        reset()
        m(510)
        m(-152)
        m(220)
        mr.run(10000)
        ml.run_angle(1500, 600)
        tmove(0.4, 60)
        wait(1400)
        hub.imu.reset_heading(-5)
        tmove(0.3, -60)
        stop()
        mr.stop()
        ml.run_angle(2000, -600)
        m(-600)
        stop()
    elif clm == "7" and Button.CENTER in hub.buttons.pressed():
        reset()
        m(255)
        pt(69, spd = 50)
        mr.run_angle(400, 150, wait=False)
        m(250)
        pt(5)
        stop()
        mr.run_angle(3200, -150)
        pt(-12)
        stop()
        mr.run_angle(4000, 92)
        tmove(0.9, 40)
        rpt(12)
        m(-170)
        mr.run_angle(4000, 90, wait=False)
        rpt(-74)
        m(-340)
        stop()
    elif clm == "8" and Button.CENTER in hub.buttons.pressed():
        reset()
        # mr.run_angle(100000, 250, wait=False)
        m(200, maxspeed=95)
        pt(90)
        m(580, maxspeed=95)
        pt(-90)
        m(300, maxspeed=95)
        pt(65)
        m(180)
        pt(-50)
        tmove(0.32, 40)
        stop()
        mr.run_angle(100000, 300)
        tmove(0.33, -40)
        rpt(50)
        m(-180)
        rpt(-50)
        m(-440, maxspeed=95)
        pt(-60)
        stop()
        tmove(0.3, 40)
        mr.run_angle(200, -450)
        m(-150)
        stop()
    wait(50)
