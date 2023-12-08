# py3.py
import random
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import time
import signal
import RPi.GPIO as GPIO


from enum import Enum
from common.server_communication import ServerComm
# 모듈 또는 파일 불러오기
from common.irSensor import InfraredSensor
from common.relaySensor import RelayModule
from common.motor import Motor, GuideMotorStep



class Step( Enum ) :
    start=0    
    second_part_irsensor_check_on = 10 
    second_part_irsensor_check_off = 15
    third_part_irsensor_check = 20
    first_go_rail= 30 
    first_stop_rail = 40
    third_part_irsensor_post = 50
    third_part_process_start = 60
    third_part_process_sleep = 70
    # third_part_sensor_measure = 80
    third_part_sensor_measure_and_endpost = 80
    third_part_process_end = 90
    servo = 100
    final_go_rail = 110
    final_stop_rail = 120

def signal_handler(sig, frame):
    print("\n프로그램 종료. GPIO 정리 중...")
    GPIO.cleanup()  # GPIO 정리
    print("GPIO 정리 완료. 프로그램 종료.")
    sys.exit(0)

# SIGINT (Ctrl+C) 시그널에 대한 핸들러 등록
signal.signal(signal.SIGINT, signal_handler)
GPIO.cleanup()  # GPIO 정리
   
pass_or_fail = GuideMotorStep.stop

SERVO_PIN_NO = 16
SONIC_IR_SENSOR_PIN_NO2 = 23
RELAY_IR_SENSOR_PIN  = 24
LIGHT_IR_SENSOR_PIN = 25
    

current_step = Step.start
running = True

sonic_ir_sensor_no2 = InfraredSensor( SONIC_IR_SENSOR_PIN_NO2 )
relay_ir_sensor = InfraredSensor( RELAY_IR_SENSOR_PIN )
light_ir_sensor = InfraredSensor( LIGHT_IR_SENSOR_PIN )
relay_module = RelayModule( 12 )
servercomm = ServerComm()
dc_motor = Motor().dc_init( 17, 27, 22 ) 
servo_motor = Motor().servo_init( SERVO_PIN_NO )



while running:
    print( "running : " + str( running ) )# 디버깅확인용
    time.sleep( 0.1 )
    SONIC_IR_SENSOR_NO2 = sonic_ir_sensor_no2.measure_ir()
    RELAY_IR_SENSOR = relay_ir_sensor.measure_ir()
    LIGHT_IR_SENSOR = light_ir_sensor.measure_ir()
    match current_step :
        case Step.start: 
            print( Step.start )
            servo_motor.doGuideMotor( GuideMotorStep.stop )
            #시작하기전에 검사할것들: 통신확인여부, 모터정렬, 센서 검수
            current_step = Step.second_part_irsensor_check_on

        # 2차 공정 적외선센서 0 -> 1 확인
        case Step.second_part_irsensor_check_on:
            
            print( Step.second_part_irsensor_check_on )
            print(SONIC_IR_SENSOR_NO2)
             # 1차 공정 두 번째 적외선 센서 값이 0이면
            if( SONIC_IR_SENSOR_NO2==0 ):
                current_step = Step.second_part_irsensor_check_off
                print("why")
            
            print(current_step)
            
        
        # 2차 공정 적외선센서 1 -> 0 확인
        case Step.second_part_irsensor_check_off:          
            print( Step.second_part_irsensor_check_off )
            
            # 2차 공정 두 번째 적외선 센서 값이 1이면
            if( SONIC_IR_SENSOR_NO2==1 ):
                
                
                # GET 통신 요청해서 답변
                if( servercomm.check_second_process()== True):
                    current_step = Step.first_go_rail
                else:
                    current_step = Step.start

                
        # 두 번째 컨베이어 벨트 구동
        case Step.first_go_rail:
            print( Step.first_go_rail )

            dc_motor.doConveyor()         
            current_step = Step.third_part_irsensor_check

        case Step.third_part_irsensor_check:
            print( Step.third_part_irsensor_check )
            print(RELAY_IR_SENSOR)
            if( RELAY_IR_SENSOR == 0 ):
                # 이온주입 공정 적외선 센서 값이 1인 상태
                # 컨베이어 벨트 정지
                current_step = Step.first_stop_rail 
            #else:             
                # 이온주입 공정 적외선 센서 값이 0인 상태 
            #    print("제품 도착 전")
            #    current_step = Step.third_part_irsensor_check

        case Step.first_stop_rail:
            # 적외선 센서 감지로 컨베이어벨트 중단
            print( Step.first_stop_rail )
            dc_motor.stopConveyor()
            current_step = Step.third_part_irsensor_post

        case Step.third_part_irsensor_post:
            print( Step.third_part_irsensor_post )
            # 서버에게 적외선 센서 감지 여부 전송
            detect_reply = servercomm.confirmationObject(3, RELAY_IR_SENSOR, "RELAY_IR_SENSOR")
            print(detect_reply)

            # 답변 중 msg 변수에 "ok" 를 확인할 시
            if( detect_reply == "ok"):
                current_step = Step.third_part_process_start
            

        case Step.third_part_process_start:
            print( Step.third_part_process_start )
            start_reply = servercomm.edsStart()

            
            # 제조 시작을 알리는 ionlmplantationStart() 함수 호출
            # 답변 중 msg 변수에 "ok" 를 확인할 시
            if( start_reply == "ok"):
                current_step = Step.third_part_process_sleep
            

        case Step.third_part_process_sleep:
            # 랜덤값 변수 대입 후 딜레이 (제조 시간 구현)
            print( Step.third_part_process_sleep )
            random_time = random.randint(4, 8)
            time.sleep(random_time)

            # 딜레이(제조)가 다 끝나면
            current_step = Step.third_part_sensor_measure_and_endpost

        # 3차 공정 품질 검사 센서값 추출
        case Step.third_part_sensor_measure_and_endpost:
            print( Step.third_part_sensor_measure_and_endpost )
            # 릴레이 모듈 값을 전도성 판단
            # 테스트로 아래 문장 주석 처리
            # relay_value = relay_module.get_relay_sensor()
            relay_value = 1
            # 릴레이 모듈 값을 서버에 전송
            end_reply = servercomm.edsEnd(relay_value)

            # 답변 따라 GuideMotorStep(good or fail) 클래스 Enum 설정
            if(end_reply == "pass"):
                pass_or_fail = GuideMotorStep.good
            else:
                pass_or_fail = GuideMotorStep.fail

            current_step = Step.servo
            
        # doGuideMotor(motor_step) 함수를 호출하여 서보모터 작동
        case Step.servo:
            print( Step.servo )
              
            servo_motor.doGuideMotor(pass_or_fail)

            current_step = Step.final_go_rail

        # 서보모터 동작 후 컨베이어벨트 구동
        case Step.final_go_rail:
            print( Step.final_go_rail)
            print( RELAY_IR_SENSOR )
            dc_motor.doConveyor()
            #######################################
            if(RELAY_IR_SENSOR == 1):
                servercomm.confirmationObject(3, RELAY_IR_SENSOR, "RELAY_IR_SENSOR")
                current_step = Step.final_stop_rail
        
         
        case Step.final_stop_rail:
            print( Step.final_stop_rail)
            # 4차 공정 적외선 센서 값으로 컨베이어벨트 정지 타이밍
            if(pass_or_fail == GuideMotorStep.good):
                if(LIGHT_IR_SENSOR == 0):
                    dc_motor.stopConveyor()
                    current_step = Step.start
            # 불량품 판정을 받았을 때 컨베이어벨트 정지 타이밍                                 
            else:
                print(pass_or_fail)
                time.sleep(5)
                dc_motor.stopConveyor()
                current_step = Step.start





##########################################################################
        
                