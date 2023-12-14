from enum import Enum
import time
import signal
import sys, os
import RPi.GPIO as GPIO
import random
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from common.motor import Motor, GuideMotorStep
from common.irSensor import InfraredSensor
from common.server_communication import ServerComm
from common.models import ProcessModel
from common.imageSensor import ImageCV

resetFlag = 0

class Step(Enum):
    start = 0
    input_part_sensor_check = 10
    wait_server_state = 20
    go_rail = 30
    photo_part_detect_sensor_check = 50
    stop_rail = 100
    #아래 추가
    photo_process_irsensor_post = 120

    photo_process = 150
    #아래 추가
    photo_process_sleep = 200

    photo_measure_and_endpost = 250
    servo_motor_drive = 300
    go_rail_next = 350
    process_check = 370
    sonic_part_detect_sensor_check = 400
    final_stop_rail = 500

# ctrl + c 누르면 프로그램 정상 종료 ( gpio 초기화 등등ㅇ)
def signal_handler(sig, frame):
    print("\n프로그램 종료. GPIO 정리 중...")
    GPIO.cleanup()  # GPIO 정리
    print("GPIO 정리 완료. 프로그램 종료.")
    sys.exit(0)

# SIGINT (Ctrl+C) 시그널에 대한 핸들러 등록
signal.signal(signal.SIGINT, signal_handler)
GPIO.cleanup()  # GPIO 정리

current_step = Step.start
running = True

# 적외선 센서 핀 번호
INPUT_IR_SENSOR_PIN = 23
IMAGE_IR_SENSOR_PIN = 24
SONIC_IR_SENSOR_PIN_NO1 = 25

First_ir_sensor = InfraredSensor(INPUT_IR_SENSOR_PIN)
Second_ir_sensor = InfraredSensor(IMAGE_IR_SENSOR_PIN)
Third_ir_sensor = InfraredSensor(SONIC_IR_SENSOR_PIN_NO1)
image_value = ImageCV()

server_comm = ServerComm()

# DCMOTOR 핀 번호
dc_enable_pin = 17
dc_input1_pin = 27
dc_input2_pin = 22
servo_pin = 16
dc_motor = Motor().dc_init(dc_enable_pin, dc_input1_pin, dc_input2_pin)
servo_motor = Motor().servo_init(servo_pin)  # 모터 핀 번호

pass_or_fail = ''

dc_motor.stopConveyor()  # DC모터 정지

while running:
    print("running : " + str(running))  # 디버깅확인용

    if(resetFlag != 0):
        dc_motor = Motor().dc_init(dc_enable_pin, dc_input1_pin, dc_input2_pin)
        servo_motor = Motor().servo_init(servo_pin)  # 모터 핀 번호
    
    time.sleep(0.1)
    INPUT_IR_SENSOR = First_ir_sensor.measure_ir()
    IMAGE_IR_SENSOR = Second_ir_sensor.measure_ir()
    SONIC_IR_SENSOR_NO1 = Third_ir_sensor.measure_ir()
    print(INPUT_IR_SENSOR)

    match current_step:
        case Step.start:  # 초기 상태, 시스템 시작
            print(Step.start)
            servo_motor.doGuideMotor(GuideMotorStep.stop)  # 서보 정렬
            
            # 시작하기전에 검사할 것들 : 통신확인여부, 모터정렬, 센서 검수
            current_step = Step.input_part_sensor_check
        
        case Step.input_part_sensor_check:
            print(Step.input_part_sensor_check)
            if (INPUT_IR_SENSOR == 0):
                # 1번핀의 감지상태
                a= server_comm.confirmationObject( 1, INPUT_IR_SENSOR, "INPUT_IR_SENSOR")
                current_step = Step.wait_server_state
            
        case Step.wait_server_state:  # 서버로부터 ok 받을 때까지 대기 (통신)
            print(Step.wait_server_state)
            result = server_comm.ready()  # get으로 물어보는 함수호출 서버에게 현재상태 물어봄
            time.sleep(1)
            ##################
            # 통신클래스에서 ok 답변을 판단해서 True로 변환 후 반환해줌
            # 따라서 result 엔 "ok" or "wait"가 아니라 True or False 가 대입
            result = True
            ##################
            if result == True:  # ok면 다음 step
                current_step = Step.go_rail
            else:
                pass
            
        case Step.go_rail:  # DC모터 구동
            print(Step.go_rail)
            dc_motor.doConveyor()
            current_step = Step.photo_part_detect_sensor_check

        case Step.photo_part_detect_sensor_check:  # 포토공정 적외선센서 감지 상태 확인
            print(Step.photo_part_detect_sensor_check)
            if (IMAGE_IR_SENSOR == 0):
                current_step = Step.stop_rail

        case Step.stop_rail:  # DC모터 정지
            print(Step.stop_rail)
            dc_motor.stopConveyor()
            current_step = Step.photo_process_irsensor_post
        #########################################################

        #포토 공정 적외선센서 감지 시 on 센서값 서버에게 전송 STEP 추가
        case Step.photo_process_irsensor_post:
            print( Step.photo_process_irsensor_post )
            # 서버에게 적외선 센서 감지 여부 전송
            start_reply = server_comm.confirmationObject(1, IMAGE_IR_SENSOR, "IMAGE_IR_SENSOR")
            
            print(start_reply)

            # 답변 중 msg 변수에 "ok" 를 확인할 시
            if( start_reply == "ok"):
                current_step = Step.photo_process
        ##############################################################



        case Step.photo_process:    # POST( 통신 )
            print(Step.photo_process)
            server_comm.photolithographyStart()    # 서버에게 이미지처리 시작하도록 알림

            if( start_reply == "ok"):
                current_step = Step.photo_process_sleep

        case Step.photo_process_sleep:
            # 랜덤값 변수 대입 후 딜레이 (제조 시간 구현)
            print( Step.photo_process_sleep )
            random_time = random.randint(4, 8)
            time.sleep(random_time)

            # 딜레이(제조)가 다 끝나면
            current_step = Step.photo_measure_and_endpost

        #################################################################

        case Step.photo_measure_and_endpost:
            print( Step.photo_measure_and_endpost )
            
            white_pixel = image_value.count_white_pixels()  # 이미지 처리 값
            pass_or_fail = server_comm.photolithographyEnd(white_pixel)  # 서버에 값을 전달(result)

            current_step = Step.servo_motor_drive
                
        case Step.servo_motor_drive:  # p or f 따라 서보모터 제어
            print( Step.photo_measure_and_endpost )

            motor_step = servo_motor.doGuideMotor(GuideMotorStep.stop)
            if (pass_or_fail == 'fail'):
                motor_step = GuideMotorStep.fail
            else:
                motor_step = GuideMotorStep.good

            servo_motor.doGuideMotor(motor_step)
            #아직 dc모터 구동 x 상황(적외선 센서 on) 이면 
            #server_comm.confirmationObject( 1, IMAGE_IR_SENSOR, "IMAGE_IR_SENSOR" )
            current_step = Step.go_rail_next

        case Step.go_rail_next:  # DC모터 재구동, 다음 단계로 이동
            print(Step.go_rail_next)                
            dc_motor.doConveyor()
            # 컨베이어 움직여야 적외선 센서가 1이 되고 off를 서버로 보낼 수 있다.
            if(IMAGE_IR_SENSOR == 1):
                server_comm.confirmationObject( 1, IMAGE_IR_SENSOR, "IMAGE_IR_SENSOR" )
                current_step = Step.process_check
            
        case Step.process_check:
            print(Step.process_check)                

            ####################################################
            if pass_or_fail == 'good':  # 불량이므로 5초 대기
                #테스트 용도로 수정함 원래는 'fail'
            ####################################################
                time.sleep(5)
                dc_motor.stopConveyor()
                current_step = Step.start
            else:
                current_step = Step.sonic_part_detect_sensor_check

        case Step.sonic_part_detect_sensor_check:  # 적외선 물체 감지
            print(Step.sonic_part_detect_sensor_check)

            if( SONIC_IR_SENSOR_NO1 == 0 ):
                    current_step = Step.final_stop_rail
                    

        case Step.final_stop_rail:  # DC모터 천천히 구동
            print(Step.final_stop_rail)
            dc_motor.stop_rail()
            # time.sleep(1)
            current_step = Step.start
            GPIO.cleanup()  # GPIO 정리
            
