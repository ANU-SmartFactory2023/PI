import RPi.GPIO as GPIO
from enum import Enum
import os
import signal
from time import sleep
import sys
import random
import sys, os
from enum import Enum
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from common.motor import Motor, GuideMotorStep
from common.irSensor import InfraredSensor
from common.lightSensor import LightSensor
from common.server_communication import ServerComm

class Step( Enum ) :    #각 스텝별 이름, 동사형으로 지을것, 무엇을 하는 스텝인지 알 수 있는 네이밍
     
    start = 0
    fourth_part_irsensor_post = 11
    fourth_part_process_start = 22
    fourth_part_process_sleep = 33
    fourth_part_sensor_measure_and_endpost = 44
    move_servo = 55
    go_rail_next_1 = 66
    stop_rail_1 = 77
    end_time = 88

pass_or_fail1 = ''
pass_or_fail2 = ''

# GPIO 핀 번호 설정
<<<<<<< HEAD
LIGHT_IR_SENSOR_PIN = 16
INPUT_IR_SENSOR_PIN = 7
SERVO_MOTOR_1_PIN = 17
SERVO_MOTOR_2_PIN = 18

# GPIO 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setup(LIGHT_IR_SENSOR_PIN, GPIO.IN)
GPIO.setup(INPUT_IR_SENSOR_PIN, GPIO.IN)
GPIO.setup(SERVO_MOTOR_1_PIN, GPIO.OUT)
GPIO.setup(SERVO_MOTOR_2_PIN, GPIO.OUT)

def signal_handler(sig, frame):
    print('프로그램을 종료합니다.')
    GPIO.cleanup()
    sys.exit(0)

# SIGINT 신호(Ctrl+C)에 대한 핸들러 등록
signal.signal(signal.SIGINT, signal_handler)


current_step = Step.start   #기본설정
running = True  
ir_sensor = InfraredSensor( INPUT_IR_SENSOR_PIN )   #센서 참조
light_sensor = LightSensor( LIGHT_IR_SENSOR_PIN )
=======
LIGHT_SENSOR_PIN = 6
LIGHT_IR_SENSOR_PIN = 7
SERVO_MOTOR_1_PIN = 17
SERVO_MOTOR_2_PIN = 18




currnet_step = Step.start   #기본설정
running = True  
ir_sensor = InfraredSensor( LIGHT_IR_SENSOR_PIN )   #센서 참조
light_sensor = LightSensor( LIGHT_SENSOR_PIN )
>>>>>>> 78b905d97fb54333d38305b4834dc9a8aafe4e28
server_comm = ServerComm()  #서버참조
servo_motor_1 = Motor().servo_init(SERVO_MOTOR_1_PIN) # 주파수 50Hz
servo_motor_2 = Motor().servo_init(SERVO_MOTOR_2_PIN)
dc_motor = Motor().dc_init(1,2,3) # DC모터

while running:
    print( "running : " + str( running ) )# 디버깅확인용
    sleep( 0.1 )

    FOURTH_IR = ir_sensor.measure_ir()
    match current_step :
        case Step.start: 
            print( Step.start )
            servo_motor_2.doGuideMotor( GuideMotorStep.stop )
            servo_motor_1.doGuideMotor( GuideMotorStep.stop )
            #시작하기전에 검사할것들: 통신확인여부, 모터정렬, 센서 검수
            current_step = Step.fourth_part_irsensor_post #다음스텝으로 이동sd

        case Step.fourth_part_irsensor_post:  
            print( Step.fourth_part_irsensor_post )
            

            if( FOURTH_IR == 1 ) :
                #서버에서 적외선 센서 감지 여부 전송
                detect_reply = server_comm.confirmationObject(4, FOURTH_IR)
                # 답변 중 msg 변수에 "ok" 를 확인할 시
                if( detect_reply == "ok"):
                    current_step = Step.fourth_part_process_start

        case Step.fourth_part_process_start:  #계산함수 시작조건 - 센서감지
            print( Step.fourth_part_process_start )
            start_reply = server_comm.euvLithographyStart() 
            # 조도센서가 임무를 수행   
            # 답변 중 msg 변수에 "ok" 를 확인할 시
            if( start_reply == "ok"):
                current_step = Step.fourth_part_process_sleep
            elif( start_reply == "fail" ) :
                current_step = Step.start

        case Step.fourth_part_process_sleep:
            # 랜덤값 변수 대입 후 딜레이 (제조 시간 구현)
            print( Step.fourth_part_process_sleep )
            random_time = random.randint(4, 8)
            sleep(random_time)
            # 딜레이(제조)가 다 끝나면
            current_step = Step.fourth_part_sensor_measure_and_endpost     

        case Step.fourth_part_sensor_measure_and_endpost:
            print( Step.fourth_part_sensor_measure_and_endpost)
            #조도센서값을 판단
            light_value = light_sensor.measure_light()
            #조도센서 값을 서버에 전송
            end_light = server_comm.euvLithographyEnd(light_value)
            if(end_light == "fail"):
                pass_or_fail2 = GuideMotorStep.reset
                pass_or_fail1 = GuideMotorStep.fail
                  
            else:
                if (end_light == "left"):
                    pass_or_fail2 = GuideMotorStep.badGrade
                    pass_or_fail1 = GuideMotorStep.good
                elif (end_light == "right"):
                    pass_or_fail2 = GuideMotorStep.goodGrade
                    pass_or_fail1 = GuideMotorStep.good

            current_step = Step.move_servo

        case Step.move_servo:
            print(Step.move_servo)
            servo_motor_1.doGuideMotor(pass_or_fail1)
            servo_motor_2.doGuideMotor(pass_or_fail2) 
            # server_comm.confirmationObject(4, FOURTH_IR,"FOURTH_IR") 
            current_step = Step.go_rail_next_1

        case Step.go_rail_next_1:
            print(Step.go_rail_next_1)
            dc_motor.doConveyor()  # 모터를 구동시킴
            if(FOURTH_IR == 0):
                current_step = Step.stop_rail_1

        case Step.stop_rail_1:
            if( pass_or_fail1 == GuideMotorStep.fail ):
                sleep(5)  # 5초 동안 대기
            else:
                sleep(8) 

            dc_motor.stopConveyor()  # 모터를 정지시킴
            current_step = Step.end_time

        case Step.end_time:
            detect_reply = server_comm.confirmationObject(4, FOURTH_IR,"allProcessFinish")
            current_step = Step.start