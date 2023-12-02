# ultrasonicSensor.py
import RPi.GPIO as GPIO
import time

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.setup_gpio()

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def cleanup_gpio(self):
        GPIO.cleanup(self.trig_pin)
        GPIO.cleanup(self.echo_pin)

    def measure_distance(self):
        try:
            GPIO.output(self.trig_pin, True)
            time.sleep(0.00001)
            GPIO.output(self.trig_pin, False)

            pulse_start = time.time()
            pulse_end = time.time()

            while GPIO.input(self.echo_pin) == 0:
                pulse_start = time.time()

            while GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distance = round(distance, 2)

            return distance

        except Exception as e:
            # 예외 처리: 예상치 못한 예외가 발생한 경우 로그를 출력하고 None을 반환
            print(f"초음파 센서 측정 중 예상치 못한 오류: {e}")
            return None

    def __del__(self):
        # 객체가 소멸될 때 GPIO 정리
        self.cleanup_gpio()