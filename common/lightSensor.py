# lightSensor.py
import RPi.GPIO as GPIO

class LightSensor:
    def __init__(self, sensor_pin, led_pin):
        """
        조도 센서 클래스 초기화 메서드.

        :param sensor_pin: 조도 센서에 연결된 GPIO 핀 번호
        """
        self.sensor_pin = sensor_pin
        self.led_pin = led_pin
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.sensor_pin, GPIO.IN)
            GPIO.setup(self.led_pin, GPIO.OUT)
            
        except Exception as e:
            print(f"GPIO 설정 중 예상치 못한 오류: {e}")

    def measure_light(self):
        """
        조도를 측정하여 반환하는 메서드.

        :return: 조도 값 (센서 입력값)
        """
        light_level = GPIO.input(self.sensor_pin)
        return light_level #0,1
    
    def turn_on_led(self):
        GPIO.output(self.led_pin, GPIO.HIGH)
        
    def turn_off_led(self):
        GPIO.output(self.led_pin, GPIO.LOW)