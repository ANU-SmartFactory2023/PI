# relaySensor.py
import RPi.GPIO as GPIO

class RelayModule:
    def __init__(self, relay_output_pin, relay_input_pin):
        """
        릴레이 모듈 클래스 초기화 메서드.

        :param relay_pin: 릴레이 모듈에 연결된 GPIO 핀 번호
        """
        self.relay_output_pin = relay_output_pin
        self.relay_input_pin = relay_input_pin
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.relay_output_pin, GPIO.OUT)
            GPIO.setup(self.relay_input_pin, GPIO.IN)
        except Exception as e:
            print(f"GPIO 설정 중 예상치 못한 오류: {e}")

    def turn_on_relay(self):
        """
        릴레이를 켜는 메서드.
        """
        try:
            GPIO.output(self.relay_output_pin, GPIO.HIGH)

            value = GPIO.input(self.relay_input_pin)

            return value

        except Exception as e:
            print(f"릴레이 켜기 중 예상치 못한 오류: {e}")