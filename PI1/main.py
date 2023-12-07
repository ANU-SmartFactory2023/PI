from enum import Enum
import time
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from common.motor import Motor, GuideMotorStep
from common.irSensor import InfraredSensor
from common.server_communication import ServerComm
from common.models import ProcessModel

class Step(Enum):
    start = 0
    input_part_sensor_check = 10
    wait_server_state = 20
    go_rail = 30
    photo_part_detect_sensor_check = 50
    stop_rail = 100
    photo_process = 150
    servo_motor_drive = 300
    go_rail_next = 350
    process_check = 370
    sonic_part_detect_sensor_check = 400
    slow_rail = 500

current_step = Step.start
running = True

# 적외선 센서 핀 번호
INPUT_IR_SENSOR_PIN = 17
PHOTO_IR_SENSOR_PIN = 18
SONICT_IR_SENSOR_PIN_NO1 = 19

First_ir_sensor = InfraredSensor(INPUT_IR_SENSOR_PIN)
Second_ir_sensor = InfraredSensor(PHOTO_IR_SENSOR_PIN)
Third_ir_sensor = InfraredSensor(SONICT_IR_SENSOR_PIN_NO1)

server_comm = ServerComm()

# DCMOTOR 핀 번호
dc_enable_pin = 20
dc_input1_pin = 21
dc_input2_pin = 22
servo_pin = 23
dc_motor = Motor().dc_init(dc_enable_pin, dc_input1_pin, dc_input2_pin)
servo_motor = Motor().servo_init(servo_pin)  # 모터 핀 번호

pass_or_fail = ''

while running:
    print("running : " + str(running))  # 디버깅확인용
    time.sleep(0.1)
    INPUT_IR_SENSOR = First_ir_sensor.measure_ir()
    IMAGE_IR_SENSOR = Second_ir_sensor.measure_ir()
    SONIC_IR_SENSOR_NO1 = Third_ir_sensor.measure_ir()

    match current_step:
        case Step.start:  # 초기 상태, 시스템 시작
            print(Step.start)
            servo_motor.doGuideMotor(GuideMotorStep.stop)  # 서보 정렬
            dc_motor.stopConveyor()  # DC모터 정지
            # 시작하기전에 검사할 것들 : 통신확인여부, 모터정렬, 센서 검수
            current_step = Step.input_part_sensor_check
        
        case Step.input_part_sensor_check:
            print(Step.input_part_sensor_check)
            if INPUT_IR_SENSOR:
                # 1번핀의 감지상태
                server_comm.confirmationObject( 1, INPUT_IR_SENSOR, "INPUT_IR_SENSOR")
                current_step = Step.wait_server_state
            
        case Step.wait_server_state:  # 서버로부터 ok 받을 때까지 대기 (통신)
            print(Step.wait_server_state)
            result = server_comm.ready()  # get으로 물어보는 함수호출 서버에게 현재상태 물어봄
            time.sleep(1)
            if result == "ok":  # ok면 다음 step
                current_step = Step.go_rail
            
        case Step.go_rail:  # DC모터 구동
            print(Step.go_rail)
            result = dc_motor.doConveyor()
            current_step = Step.photo_part_detect_sensor_check

        case Step.photo_part_detect_sensor_check:  # 포토공정 적외선센서 감지 상태 확인
            print(Step.photo_part_detect_sensor_check)
            if IMAGE_IR_SENSOR:
                current_step = Step.stop_rail

        case Step.stop_rail:  # DC모터 정지
            print(Step.stop_rail)
            dc_motor.stopConveyor()
            current_step = Step.photo_process
        
        case Step.photo_process:    # POST( 통신 )
            print(Step.photo_process)
            server_comm.photolithographyStart()    # 서버에게 이미지처리 시작하도록 알림
            result = IMAGE_IR_SENSOR  # 이미지 처리 값
            pass_or_fail = server_comm.photolithographyEnd(result)  # 서버에 값을 전달(result)

            current_step = Step.servo_motor_drive
                
        case Step.servo_motor_drive:  # p or f 따라 서보모터 제어
            motor_step = servo_motor.doGuideMotor(GuideMotorStep.stop)
            if (pass_or_fail == 'fail'):
                motor_step = GuideMotorStep.fail
            else:
                motor_step = GuideMotorStep.good

            servo_motor.doGuideMotor(motor_step)
            server_comm.confirmationObject( 1, IMAGE_IR_SENSOR, "IMAGE_IR_SENSOR" )
            current_step = Step.go_rail_next

        case Step.go_rail_next:  # DC모터 재구동, 다음 단계로 이동
            print(Step.go_rail)                
            result = dc_motor.doConveyor()
            current_step = Step.process_check
            
        case Step.process_check:
            if pass_or_fail == 'fail':  # 불량이므로 5초 대기
                time.sleep(5)
                dc_motor.stopConveyor()
                current_step = Step.start
            else:
                current_step = Step.sonic_part_detect_sensor_check

        case Step.sonic_part_detect_sensor_check:  # 적외선 물체 감지
            print(Step.sonic_part_detect_sensor_check)
            if SONIC_IR_SENSOR_NO1:
                server_comm.confirmationObject( 2, SONIC_IR_SENSOR_NO1 )
                current_step = Step.slow_rail

        case Step.slow_rail:  # DC모터 천천히 구동
            print(Step.slow_rail)
            result = dc_motor.slowConveyor()
            current_step = Step.start