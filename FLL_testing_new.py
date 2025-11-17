
#Movement z FLL
#dajaky algoritmus na pohyb okolo sviecok alebo daco take
#ked uvidi dialkovy sviecku otoc sa doprava 90, chod dopredu dialkovy * 8 (treba s tym cislom experimentovat), a zacni tocit vrtulu na forklifte hore a postupne pojde dole 
#potom nech pocuva tolko kolko isiel otoci sa dolava 90 a pokracuje v algoritme

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.parameters import Color as Col
from pybricks.tools import wait as pb_wait, wait
import util  # your util.py for display/font/etc.

# --- HUB INSTANCE ---
shub = PrimeHub()
shub.system.set_stop_button(None)

# --- VERSION INFO ---
class version:
    __version__ = "1.0.0"

# --- DEBUG TOGGLES ---
BOMB_GYRO = False
BOMB_COLOR = False

# --- ROBOT CONSTANTS ---
WHEEL_DIAMETER = 56  # mm
WHEEL_CIRROT = (56 / 10) * 3.1416  # approximate π
global loop 
loop = True
# --- PORTS ---
class PORT:
    RDM = Port.A  # Right Driving Motor
    LDM = Port.B  # Left Driving Motor
    RMM = Port.F  # Right Medium Motor
    LMM = Port.C  # Left Medium Motor
    LCS = Port.E  # Color Sensor
    DS = Port.D   # Distance Sensor
def calc_gear_ratio(gears):
    """
    Calculate gear ratio from a list of gear sizes.
    Example: [12, 20] means 12 driving 20, ratio = 12/20 = 0.6
    """
    if not gears or len(gears) < 2:
        return 1.0
    ratio = 1.0
    for i in range(len(gears) - 1):
        ratio *= gears[i] / gears[i + 1]
    return ratio
# --- BUTTON HELPERS ---
def is_button(side):
    pressed = shub.buttons.pressed()
    if side == "left":
        return Button.LEFT in pressed
    elif side == "right":
        return Button.RIGHT in pressed
    elif side == "center":
        return Button.CENTER in pressed
    return False

def led(color: str):
    shub.light.on(color)

def display_number(n: int):
    shub.display.text(str(n))

# --- STOP FLAG ---
stop_requested = False

def request_stop():
    """Mark stop requested (called when user presses center during run)."""
    global stop_requested
    stop_requested = True

def reset_stop():
    """Clear stop flag (called before running a new program)."""
    global stop_requested
    stop_requested = False

def should_stop():
    """Return True if stop requested OR center button pressed."""
    return stop_requested or (Button.CENTER in shub.buttons.pressed())



# --- SENSORS ---
class Gyro:
    def __init__(self):
        self.value = 0

    def angle(self):
        if BOMB_GYRO:
            return 0
        self.value = shub.imu.heading()
        return self.value

    def reset(self):
        shub.imu.reset_heading(0)

class Color:
    def __init__(self, port):
        self.sensor = ColorSensor(port)

    def get(self):
        if BOMB_COLOR:
            return 0
        return self.sensor.reflection()
class API:
    def __init__(self):
        # PID constants
        self.Kp = 2.6
        self.Ki = 0.1
        self.Kd = 0.3
        self.Kc = 0.03

        # Sensors
        self.gyro = Gyro()
        self.lCs = Color(PORT.LCS)
        # self.rCs = Color(PORT.RCS)
        self._async_motors = []
        # Motors
        self.LM = Motor(PORT.LDM, Direction.CLOCKWISE)
        self.RM = Motor(PORT.RDM, Direction.COUNTERCLOCKWISE)
        self.LMM = Motor(PORT.LMM, Direction.COUNTERCLOCKWISE)
        self.RMM = Motor(PORT.RMM, Direction.CLOCKWISE)

    def write_info(self):
        V_MIN = 6600
        V_MAX = 8200
        voltage_mV = shub.battery.voltage()
        battery_percent = int((voltage_mV - V_MIN) / (V_MAX - V_MIN) * 100)
        battery_percent = max(0, min(100, battery_percent))
        print("Spike Hub version:", version.__version__)
        print(f"Battery: {battery_percent}% ({voltage_mV} mV)")

    def reset_for_start(self):
        self.gyro.reset()

    # --- MOVEMENT FUNCTIONS ---
    def straight(self, target, speed=60, accel=False, decel=False):
        self.wait_async()
        speed_change_rate = speed * 0.01005
        decel_distance = 12 + (12 * ((speed - 50) / 100))

        proportional = integral = derivative = 0
        lasterr = 0
        timer = speed
        phase = 1 # 0 = accel, 1 = full, 2 = decel

        if accel:
            timer = 0
            phase = 0

        self.reset_for_start()
        pb_wait(50)  # short warm-up
        start_angle = (self.LM.angle() + self.RM.angle()) / 2

        while abs((self.LM.angle() + self.RM.angle()) / 2 - start_angle) < abs(target):
            pb_wait(2)

            if should_stop() or is_button("center"):
                print("Straight interrupted!")
                self.LM.stop()
                self.RM.stop()
                return

            # --- PID ---
            heading = self.gyro.angle()
            proportional = -heading if speed > 0 else heading  # flipped direction
            integral += proportional
            derivative = proportional - lasterr
            lasterr = proportional

            steering = self.Kp * proportional + self.Ki * integral + self.Kd * derivative
            steering = max(-100, min(100, steering))

            # --- Accel/decel control ---
            if phase == 0:
                timer += speed_change_rate
            elif phase == 2 and timer > 15:
                timer -= speed_change_rate
            if phase == 0 and timer >= speed:
                phase = 1
                timer = speed
            if phase == 1 and decel and abs(target) - abs((self.LM.angle() + self.RM.angle()) / 2 - start_angle) <= decel_distance:
                phase = 2

            # --- Run motors ---
            self.LM.run(timer - steering)  # swapped signs here
            self.RM.run(timer + steering)  # swapped signs here

            pb_wait(2)

        # --- Stop cleanly ---
        self.LM.stop()
        self.RM.stop()

    # TURN WITH GYRO + ACCEL/DECEL
    def turn(self, target, speed=60, accel=False, decel=False):
        self.wait_async()
        speed_change_rate = speed * 0.006
        decel_distance = 8 * (speed / 50)
        timer = speed
        phase = 1
        if accel:
            timer = 0
            phase = 0

        self.reset_for_start()
        pb_wait(50)
        while abs(self.gyro.angle()) < abs(target):
            if should_stop():
                print("Turn interrupted!")
                self.LM.stop()
                self.RM.stop()
                return

            if phase == 0:
                timer += speed_change_rate
            elif phase == 2 and timer > 15:
                timer -= speed_change_rate
            if phase == 0 and timer >= speed:
                phase = 1
                timer = speed
            if phase == 1 and decel and abs(target) - abs(self.gyro.angle()) <= decel_distance:
                phase = 2

            direction = 1 if target > 0 else -1
            self.LM.run(timer * direction)
            self.RM.run(-timer * direction)

            pb_wait(2)
            if should_stop():
                self.LM.stop()
                self.RM.stop()
                return

        self.LM.stop()
        self.RM.stop()

    # MEDIUM MOTOR WITH GEAR SUPPORT
    def _async(self, enabled=True):
        """Enable or disable async motor mode."""
        self.async_mode = enabled

    def wait_async(self):
        """Wait for all async motors to stop before continuing."""
        for motor in self._async_motors:
            motor.done()
        self._async_motors.clear()

    def medium(self, target_deg, speed=100, port=PORT.RMM, gears=None):
        # Determine motor
        if port == PORT.RMM:
            motor = self.RMM
        elif port == PORT.LMM:
            motor = self.LMM
        else:
            print("INCORRECT PORT: medium " + str(port) +
                  " " + str(target_deg) + "deg " + str(speed))
            return

        # Calculate ratio
        ratio = 1.0
        if gears and len(gears) > 1:
            for i in range(len(gears) - 1):
                ratio *= gears[i] / gears[i + 1]

        motor_deg = target_deg / ratio
        motor_speed = speed / ratio

        # Run async or sync
        if getattr(self, "async_mode", False):
            motor.run_angle(motor_deg, motor_speed, wait=False)
            self._async_motors.append(motor)
        else:
            motor.run_angle(motor_deg, motor_speed, wait=True)
    # LINE ASSISTED STRAIGHT (PID + COLOR)
    def lags(self, target, speed=60, portside="rr", accel=False, decel=False):
        speed_change_rate = speed * 0.01005
        decel_distance = 12 + (12 * ((speed - 50) / 100))
        perfect_reflectance = 50

        proportional = integral = derivative = 0
        lasterr = 0
        timer = speed
        phase = 1
        if accel:
            timer = 0
            phase = 0

        self.reset_for_start()

        distance_done = 0
        step_cm = 1
        pb_wait(50)
        while distance_done < target:
            if should_stop():
                print("Line-follow interrupted!")
                self.LM.stop()
                self.RM.stop()
                return

            heading = self.gyro.angle()
            proportional = -heading if speed > 0 else heading
            integral += proportional
            derivative = proportional - lasterr
            lasterr = proportional

            if portside == "rr":
                reflectance = self.lCs.get() - perfect_reflectance
            elif portside == "rl":
                reflectance = perfect_reflectance - self.rCs.get()
            elif portside == "ll":
                reflectance = perfect_reflectance - self.lCs.get()
            elif portside == "lr":
                reflectance = self.rCs.get() - perfect_reflectance
            else:
                reflectance = 0

            steering = (self.Kp*proportional +
                        self.Ki*integral +
                        self.Kd*derivative +
                        self.Kc*reflectance)
            steering = max(-100, min(100, steering))

            if phase == 0:
                timer += speed_change_rate
            elif phase == 2 and timer > 15:
                timer -= speed_change_rate
            if phase == 0 and timer >= speed:
                phase = 1
                timer = speed
            if phase == 1 and decel and target - distance_done <= decel_distance:
                phase = 2

            self.LM.run(timer + steering)
            self.RM.run(timer - steering)

            pb_wait(2)
            if should_stop():
                self.LM.stop()
                self.RM.stop()
                return

            distance_done += step_cm

        self.LM.stop()
        self.RM.stop()

# --- EXAMPLE PROGRAMS ---
def exec_0(api):
    api.straight(1000, 1000)
    api.straight(1300, -800)

def exec_1(api):
    api.straight(2000, 500)

def exec_2(api):
    api.turn(3600, 950)

def exec_3(api):
    api.straight(600, 500)
    api._async(True)
    api.medium(-1200, 500, PORT.RMM)
    api.medium(1200, 500, PORT.LMM)
    api._async(False)
    api.straight(600, -500)

def exec_4(api):
    api.turn(90, 150)

def exec_5(api):
    api.straight(3600, 500)
    api.turn(-90, 200)

def exec_6(api):
    pass
def exec_7(api):
    while True:
        pb_wait(2)
        print(api.gyro.angle())

def exec_8(api):
    api.LMM.run_time(500, 1800)#reset forklift
    api.LMM.run_time(-500, 1800)#reset forklift
    #api.RMM.run_time(-1000, 8000)
    dialkovy = UltrasonicSensor(PORT.DS)#nastavenie USC
    colorn = ColorSensor(PORT.LCS)#nastavenie CS
    while dialkovy.distance() > 500:# opakuj kym USC neni pod 50cm
        while colorn.reflection() <= 20: #opakuj kym neni CS reflekcia pod 10%
            api.straight(50, 200)# chod do predu 50 stupnov
            if dialkovy.distance() < 500: #ked je USC pod 50cm
                dialka = dialkovy.distance() / 1.8#nastav cuvanie dialku
                api.straight(200, 400)#chod do predu 200 stupnov
                api.turn(90, -100)#otoc sa 90 stupnov do prava
                while colorn.reflection() < 20:#kym CS reflekcia je menej nez 20%
                    wait(20)#cakaj 20 ms
                    api.straight(20,100)#chod do predu
                for i in range(3):#opakuj 3 krat
                    api.RMM.run_time(-1000, 4000)#toc vrtulov
                    api.LMM.run_time(500, 800)#dvihni forklift
                api.LMM.run_time(-500, 1800)#zloz forklift uplne dole
                api.straight(dialka, -500)#cuvaj (dialka)
                pb_wait(200)#cakaj 200ms
                api.turn(90, 100)#otoc sa 90 stupnov dolava
                pb_wait(200)#pockaj 200ms
                api.straight(500, 300)#chod do predu 500 stupnov
        api.straight(50, -200)#chod do predu 50 stupnov
        api.turn(-90, 200)#otoc sa dolava 90 stupnov
#ked pojde do predu a neuvidi ciernu urcity cas tak sa zacni tocit do prava kym neuvidis sviecku, potom chod ku nej sfukni a skonci program (toto je posledne)

def koniec_jazdy(api):
    dialkovy = UltrasonicSensor(PORT.DS)
    colorn = ColorSensor(PORT.LCS)
    while dialkovy.distance() >= 200:
        api.turn(20, 500)
    while colorn.reflection() < 20:
        wait(20)
        api.straight(20,100)
    for i in range(3):
        api.RMM.run_time(-1000, 4000)
        api.LMM.run_time(500, 800)
    shub.system.shutdown()


def exec_9(api):
    api.straight(350, 300)
    pb_wait(200)
    api.turn(90, 150)
    pb_wait(200)
    api.straight(380,300)
    pb_wait(200)
    api.turn(-90, 150)
    pb_wait(200)
    api.straight(780, 200)
    pb_wait(200)
    api.turn(90, 150)
    pb_wait(200)
    api.straight(750, 500)
    pb_wait(100)
    api.turn(-45, 150)
    pb_wait(200)
    api.straight(750, 300)
    pb_wait(200)
    api.turn(45, 150)
    pb_wait(200)
    sensor = ColorSensor(PORT.LCS)
    while not sensor.color() == Col.RED:
        api.straight(50, 300)


def monitor_stop():
    if is_button("center"):
        request_stop()

# --- MAIN LOOP ---
from pybricks.tools import hub_menu

def main():
    api = API()
    api.write_info()

    # Start at program 0
    last_selected = "0"

    while True:
        if is_button("right") and is_button("left"):
            break
        selected = hub_menu("0","1","2","3","4","5","6","7","8","9")
        # If user presses back (center while in menu), we still remember last
        if selected is None:
            selected = last_selected
        else:
            last_selected = selected

        reset_stop()

        # Run chosen program
        try:
            if selected == "0": exec_0(api)
            elif selected == "1": exec_1(api)
            elif selected == "2": exec_2(api)
            elif selected == "3": exec_3(api)
            elif selected == "4": exec_4(api)
            elif selected == "5": exec_5(api)
            elif selected == "6": exec_6(api)
            elif selected == "7": exec_7(api)
            elif selected == "8": exec_8(api)
            elif selected == "9": exec_9(api)
            if is_button("center"):
                request_stop()
        except SystemExit:
            # In case something calls sys.exit(), ignore and go back to menu
            pass
if __name__ == "__main__":
    main()

