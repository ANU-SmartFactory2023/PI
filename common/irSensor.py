# irSensor.py
import RPi.GPIO as GPIO

class InfraredSensor:
    def __init__(self, ir_pin):
        """
        적외선 센서 클래스 초기화 메서드.

        :param ir_pin: 적외선 센서에 연결된 GPIO 핀 번호
        """
        self.ir_pin = ir_pin
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.ir_pin, GPIO.IN)
        except Exception as e:
            print(f"GPIO 설정 중 예상치 못한 오류: {e}")

    def measure_ir(self):
        """
        적외선 센서 값을 측정하여 반환하는 메서드.

        :return: 적외선 센서 값 (센서 입력값)
        """
        ir_value = GPIO.input(self.ir_pin)
        return ir_value