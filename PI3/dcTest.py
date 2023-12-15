import RPi.GPIO as GPIO
import time

# GPIO 핀 설정
ENA = 17  # PWM 핀
IN1 = 27
IN2 = 22

# 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# PWM 객체 생성
pwm_motor = GPIO.PWM(ENA, 100)  # 100Hz 주파수

# 모터 정회전
GPIO.output(IN1, GPIO.HIGH)
GPIO.output(IN2, GPIO.LOW)

# PWM 시작
pwm_motor.start(0)

try:
    while True:
        # 속도 조절 (0~100 사이의 값을 사용)
        speed = 50
        pwm_motor.ChangeDutyCycle(speed)
        time.sleep(5)  # 일정 시간 동안 유지한 후 종료

except KeyboardInterrupt:
    # 프로그램 종료 시 GPIO 정리
    pwm_motor.stop()
    GPIO.cleanup()
