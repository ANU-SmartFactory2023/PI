#motor.py
import RPi.GPIO as GPIO
from time import sleep
from enum import Enum

class GuideMotorStep(Enum):
    reset = 0
    stop = 1
    fail = 2
    good = 3
    badGrade = 4
    goodGrade = 5
    re = 6

class Motor:
    def servo_init(self, servo_pin):

        # 서보 모터 초기화
        self.servo_pin = servo_pin
        GPIO.setup(self.servo_pin, GPIO.OUT)

        self.pwm_servo = GPIO.PWM(self.servo_pin, 50)  # 주파수 50Hz로 PWM 설정
        self.pwm_servo.start(0)  # 초기 듀티 사이클은 0으로 설정
        # 5 서보모터 45, 135도 구동
        self.final_angle = {"goodGrade": 45, "badGrade": 135}

        return self

    def dc_init(self, dc_enable_pin, dc_input1_pin, dc_input2_pin):
        # # DC 모터 초기화
        self.dc_enable_pin = dc_enable_pin
        self.dc_input1_pin = dc_input1_pin
        self.dc_input2_pin = dc_input2_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dc_enable_pin, GPIO.OUT)
        GPIO.setup(self.dc_input1_pin, GPIO.OUT)
        GPIO.setup(self.dc_input2_pin, GPIO.OUT)

        self.pwm_dc = GPIO.PWM(self.dc_enable_pin, 1000)  # 주파수 1000Hz로 PWM 설정
        self.pwm_dc.start(0)  # 초기 듀티 사이클은 0으로 설정

        return self


    def changeDutyCycle(self, angle):
        # 주어진 각도에 따른 duty cycle 계산
        return (angle / 18.0) + 2.5
    
    def doGuideMotor(self, step):
        if step == GuideMotorStep.reset:
            self.pwm_servo.ChangeDutyCycle(self.changeDutyCycle(0))
        elif step == GuideMotorStep.stop:
            self.pwm_servo.ChangeDutyCycle(self.changeDutyCycle(90))
        elif step == GuideMotorStep.fail:
            self.pwm_servo.ChangeDutyCycle(self.changeDutyCycle(135))
        elif step == GuideMotorStep.good:
            self.pwm_servo.ChangeDutyCycle(self.changeDutyCycle(180))
        elif step == GuideMotorStep.badGrade:
            self.pwm_servo.ChangeDutyCycle(self.changeDutyCycle(45))
        elif step == GuideMotorStep.goodGrade:
            self.pwm_servo.ChangeDutyCycle(self.changeDutyCycle(135))
        elif step == GuideMotorStep.re:
            self.pwm_servo.ChangeDutyCycle(self.changeDutyCycle(0))

    def doConveyor(self):
        # 속도 설정
        speed = 100

        duty_cycle = speed * 1.1 + 10
        self.pwm_dc.ChangeDutyCycle(duty_cycle)
        # 정방향 회전
        GPIO.output(self.dc_input1_pin, GPIO.HIGH)
        GPIO.output(self.dc_input2_pin, GPIO.LOW)

    def slowConveyor(self):
        GPIO.output(self.dc_input1_pin, GPIO.HIGH)
        GPIO.output(self.dc_input2_pin, GPIO.LOW)
        speed = 50      # 속도 설정
        duration = 3    # 3초간 50으로 구동
        self.doConveyor(speed)
        sleep(duration)
        self.stopConveyor()

    def stopConveyor(self):
        # 정지
        GPIO.output(self.dc_input1_pin, GPIO.LOW)
        GPIO.output(self.dc_input2_pin, GPIO.LOW)

    def cleanup(self):
        # 정리
        self.pwm_dc.stop()
        self.pwm_servo.stop()
        GPIO.cleanup()