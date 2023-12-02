# py3.py
import random
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import time


from enum import Enum
from common.server_communication import ServerComm
# 모듈 또는 파일 불러오기
from common.irSensor import InfraredSensor
from common.relaySensor import RelayModule
from common.motor import Motor, GuideMotorStep



class Step( Enum ) :
    start=0    
    second_part_irsensor_check = 10 
    third_part_irsensor_check = 20
    go_rail= 30 
    stop_rail = 40
    third_part_irsensor_post = 50
    third_part_process_start = 60
    third_part_process_sleep = 70
    third_part_sensor_measure = 80
    third_part_process_end = 90
    servo = 100
   
pass_or_fail = ''

SERVO_PIN = 16
SECOND_PART_SENSOR_PIN_NO = 17
THIRD_PART_SENSOR_PIN_NO = 18
    

currnet_step = Step.start
running = True

ir_sensor_2 = InfraredSensor( SECOND_PART_SENSOR_PIN_NO )
ir_sensor_3 = InfraredSensor( THIRD_PART_SENSOR_PIN_NO )
relay_module = RelayModule( 12 )
servercomm = ServerComm()
dc_motor = Motor().dc_init( 1, 2, 3 ) 
servo_motor = Motor().servo_init( SERVO_PIN )


while running:
    print( "running : " + str( running ) )# 디버깅확인용
    time.sleep( 0.1 )
    SECOND_IR = ir_sensor_2.measure_ir()
    THIRD_IR = ir_sensor_3.measure_ir()
    match currnet_step :
        case Step.start: 
            print( Step.start )
            servo_motor.doGuideMotor( GuideMotorStep.stop )
            #시작하기전에 검사할것들: 통신확인여부, 모터정렬, 센서 검수
            currnet_step = Step.second_part_irsensor_check
        
        case Step.second_part_irsensor_check:
            print( Step.second_part_irsensor_check )
            if( sensor.get_ir_sensor( SECOND_PART_SENSOR_PIN_NO )==0 ):
                # 2차 공정 두 번째 적외선 센서 값이 0이면
                current_step = Step.go_rail
            else:
                
        # 두 번째 컨베이어 벨트 구동
        case Step.go_rail:
            print( Step.go_rail )
            dc_motor.doConveyor();           
            currnet_step = third_part_irsensor_check

        case Step.third_part_irsensor_check:
            print( Step.third_part_irsensor_check )
            if( THIRD_IR == 1 ):
                # 이온주입 공정 적외선 센서 값이 1인 상태
                # 컨베이어 벨트 정지
                currnet_step = stop_rail 
            else:             
                # 이온주입 공정 적외선 센서 값이 0인 상태 
                print("제품 도착 전")
                currnet_step = Step.third_part_irsensor_check

        case Step.stop_rail:
            # 적외선 센서 감지로 컨베이어벨트 중단
            print( Step.stop_rail )
            motor.stopConveyor()
            currnet_step = Step.third_part_irsensor_post

        case Step.third_part_irsensor_post:
            print( third_part_irsensor_post )
            # 서버에게 적외선 센서 감지 여부 전송
            decect_reply = confirmationObject(3, THIRD_IR)

            # 답변 중 msg 변수에 "ok" 를 확인할 시
            if( detect_reply == "ok"):
                current_step = Step.third_part_process_start
            else:
                current_step = Step.third_part_irsensor_post

        case Step.third_part_process_start:
            print( third_part_process_start )
            start_reply = ionlmplantationStart()

            # 제조 시작을 알리는 ionlmplantationStart() 함수 호출
            # 답변 중 msg 변수에 "ok" 를 확인할 시
            if( start_reply == "ok"):
                current_step = Step.third_part_process_sleep
            else:
                current_step = Step.third_part_process_start

        case Step.third_part_process_sleep:
            # 랜덤값 변수 대입 후 딜레이 (제조 시간 구현)
            print( Step.third_part_process_sleep )
            random_time = random.randint(4, 8)
            sleep(random_time)

            # 딜레이(제조)가 다 끝나면
            current_step = Step.third_part_sensor_measure

        # 3차 공정 품질 검사 센서값 추출
        case Step.third_part_sensor_measure_and_endpost:
            print( Step.third_part_sensor_measure )
            # 릴레이 모듈 값을 전도성 판단
            relay_value = sensor.get_relay_sensor()
            # 릴레이 모듈 값을 서버에 전송
            end_reply = ionlmplantationEnd(relay_value)

            # 답변 따라 GuideMotorStep(good or fail) 클래스 Enum 설정
            if(end_reply = "pass"):
                pass_or_fail = good
            else:
                pass_or_fail = fail

            current_step = Step.servo


        # doGuideMotor(motor_step) 함수를 호출하여 서보모터 작동
        case Step.servo:
            print( Step.servo )
            motor_step = GuideMotorStep.stop    #기본 stop
            if(pass_or_fail == "good"):
                motor_step = motor.GuideMotorStep.good
            else:
                motor_step = motor.GuideMotorStep.fail
        
            doGuideMotor(motor_step)

            currnet_step = Step.go_rail

##########################################################################