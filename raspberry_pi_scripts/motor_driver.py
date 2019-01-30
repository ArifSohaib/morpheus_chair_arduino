import RPi.GPIO as GPIO
import time



class MotorDriver(object):

    def __init__(self):

        self.PIN = 18
        self.PWMA1 = 6
        self.PWMA2 = 13
        self.PWMB1 = 20
        self.PWMB2 = 21
        self.D1 = 12
        self.D2 = 26

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.PIN, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(self.PWMA1, GPIO.OUT)
        GPIO.setup(self.PWMA2, GPIO.OUT)
        GPIO.setup(self.PWMB1, GPIO.OUT)
        GPIO.setup(self.PWMB2, GPIO.OUT)
        p1 = GPIO.PWM(self.D1, 500)
        p2 = GPIO.PWM(self.D2, 500)
        p1.start(50)
        p2.start(50)

    def set_motor(self, A1, A2, B1, B2):
        GPIO.output(self.PWMA1, A1)
        GPIO.output(self.PWMA2, A2)
        GPIO.output(self.PWMB1, B1)
        GPIO.output(self.PWMB2, B2)

    def forward(self):
        self.set_motor(1, 0, 1, 0)

    def stop(self):
        self.set_motor(0, 0, 0, 0)


    def reverse(self):
        self.set_motor(0, 1, 0, 1)


    def left(self):
        self.set_motor(1, 0, 0, 0)


    def right(self):
        self.set_motor(0, 0, 1, 0)


def motor_driver_test():
    """
    Test where we go forewards, backwards, left, right and stop.
    :return:
    """
    motor = MotorDriver()

    motor.stop()
    print("pre-stop")
    time.sleep(0.5)

    motor.forward()
    print("forward")
    time.sleep(2.0)

    motor.reverse()
    print("reverse")
    time.sleep(2.0)

    motor.left()
    print("left")
    time.sleep(2.0)

    motor.right()
    print("right")
    time.sleep(2.0)

    motor.stop()
    print("stop")
    time.sleep(2.0)

    GPIO.cleanup()

if __name__ == "__main__":
    print("Starting Motor DRiver Test")
    motor_driver_test()
    print("Starting Motor DRiver Test...END")

