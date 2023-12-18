# py3.py
import random
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import time
import signal
import RPi.GPIO as GPIO
from enum import Enum

# 모듈 또는 파일 불러오기
from common.server_communication import ServerComm

from common.irSensor import InfraredSensor
from common.relaySensor import RelayModule
from common.lightSensor import LightSensor
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
########### euv 공정 ###############
    fourth_part_irsensor_post = 11
    fourth_part_process_start = 22
    fourth_part_process_sleep = 33
    fourth_part_sensor_measure_and_endpost = 44
    move_servo = 55
    go_rail_next_1 = 66
    stop_rail_1 = 77
    end_time = 88


def signal_handler(sig, frame):
    print("\n프로그램 종료. GPIO 정리 중...")
    GPIO.cleanup()  # GPIO 정리
    print("GPIO 정리 완료. 프로그램 종료.")
    sys.exit(0)

# SIGINT (Ctrl+C) 시그널에 대한 핸들러 등록
signal.signal(signal.SIGINT, signal_handler)
GPIO.cleanup()  # GPIO 정리
   
pass_or_fail_eds = ''
pass_or_fail_euv = ''
pass_or_fail_grade = ''
# GPIO 핀 번호 설정
# 서보 모터 핀
SERVO_EDS_PIN_NO = 21
SERVO_EUV_PIN = 20
SERVO_GRADE_PIN = 16
# 로봇팔 신호
RELAY_IN_ARM_PIN_NO = 2
RELAY_OUT_ARM_PIN_NO = 3

# DC모터 핀
dc_enable_pin = 17
dc_input1_pin = 27
dc_input2_pin = 22

# 적외선 센서 핀
SONIC_IR_SENSOR_PIN_NO2 = 4
RELAY_IR_SENSOR_PIN  = 15
LIGHT_IR_SENSOR_PIN = 18

# # 릴레이 센서 핀
RELAY_OUPUT_PIN_NO = 1
RELAY_INPUT_PIN_NO = 12

# 조도센서 LED핀
LIGHT_SENSOR_PIN = 19
LED_PIN = 26
    

current_step = Step.start
running = True

sonic_ir_sensor_no2 = InfraredSensor( SONIC_IR_SENSOR_PIN_NO2 )
relay_ir_sensor = InfraredSensor( RELAY_IR_SENSOR_PIN )
light_ir_sensor = InfraredSensor( LIGHT_IR_SENSOR_PIN )
relay_module = RelayModule( RELAY_OUPUT_PIN_NO, RELAY_INPUT_PIN_NO)
light_sensor = LightSensor(LIGHT_SENSOR_PIN, LED_PIN)
servercomm = ServerComm()
dc_motor = Motor().dc_init( dc_enable_pin, dc_input1_pin, dc_input2_pin ) 
servo_eds_motor = Motor().servo_init( SERVO_EDS_PIN_NO )
servo_euv_motor = Motor().servo_init(SERVO_EUV_PIN)  # 주파수 50Hz
servo_grade_motor = Motor().servo_init(SERVO_GRADE_PIN)

output_arm_servo = Motor().servo_init( RELAY_OUT_ARM_PIN_NO )
input_arm_servo = Motor().servo_init( RELAY_IN_ARM_PIN_NO )


while running:
    print( "running : " + str( running ) )# 디버깅확인용
    time.sleep( 0.1 )
    SONIC_IR_SENSOR_NO2 = sonic_ir_sensor_no2.measure_ir()
    RELAY_IR_SENSOR = relay_ir_sensor.measure_ir()
    LIGHT_IR_SENSOR = light_ir_sensor.measure_ir()
    LIGHT_SENSOR = light_sensor.measure_light()
    
    light_value = light_sensor.measure_light()
    print(light_value)
    
    match current_step :
        case Step.start: 
            print( Step.start )
            servo_eds_motor.doGuideMotor( GuideMotorStep.stop )
            servo_grade_motor.doGuideMotor(GuideMotorStep.stop)
            servo_euv_motor.doGuideMotor(GuideMotorStep.stop)
            output_arm_servo.doGuideMotor(GuideMotorStep.out_arm_open)
            input_arm_servo.doGuideMotor(GuideMotorStep.in_arm_open)
            #시작하기전에 검사할것들: 통신확인여부, 모터정렬, 센서 검수
            current_step = Step.second_part_irsensor_check_on

            # 2차 공정 적외선센서 0 -> 1 확인
        case Step.second_part_irsensor_check_on:
                
            print( Step.second_part_irsensor_check_on )
            
            # 1차 공정 두 번째 적외선 센서 값이 0이면
            if( SONIC_IR_SENSOR_NO2==0 ):
                current_step = Step.second_part_irsensor_check_off
                
                       
                
            
            # 2차 공정 적외선센서 1 -> 0 확인
        case Step.second_part_irsensor_check_off:          
            print( Step.second_part_irsensor_check_off )
                
            # 2차 공정 두 번째 적외선 센서 값이 1이면
            if( SONIC_IR_SENSOR_NO2==1 ):
                    
                second_Rail_Start = servercomm.check_second_process()
                print(second_Rail_Start)

                # GET 통신 요청해서 답변
                if( second_Rail_Start== True):
                    current_step = Step.first_go_rail
                else:
                    current_step = Step.start
                    
        # 두 번째 컨베이어 벨트 구동
        case Step.first_go_rail:
            print( Step.first_go_rail )

            dc_motor.doConveyor()     
            ## 두 번째 컨베이어벨트 구동 때 로봇 암까지 같이 구동  
            output_arm_servo.doGuideMotor(GuideMotorStep.out_arm_check)
            input_arm_servo.doGuideMotor(GuideMotorStep.in_arm_check)
            #########@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#################
            current_step = Step.third_part_irsensor_check

        case Step.third_part_irsensor_check:
            print( Step.third_part_irsensor_check )
            
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
            relay_value = relay_module.turn_on_relay()
            print(relay_value)
            #relay_value = 1
            # 릴레이 모듈 값을 서버에 전송
            end_reply = servercomm.edsEnd(relay_value)

            print (end_reply)

                # 답변 따라 GuideMotorStep(good or fail) 클래스 Enum 설정
            if(end_reply == "pass"):
                pass_or_fail_eds = GuideMotorStep.good
            else:
                pass_or_fail_eds = GuideMotorStep.fail
                

            current_step = Step.servo
                
        # doGuideMotor(motor_step) 함수를 호출하여 서보모터 작동
        # 로봇팔도 진행 방향 오픈
        ###################3@@@@@@@@@@@@@@@@@@@@@@@@@
        case Step.servo:
            print( Step.servo )
            
            output_arm_servo.doGuideMotor(GuideMotorStep.out_arm_open)
            input_arm_servo.doGuideMotor(GuideMotorStep.in_arm_open)
            servo_eds_motor.doGuideMotor(pass_or_fail_eds)

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
            if(pass_or_fail_eds == GuideMotorStep.good):
                if(LIGHT_IR_SENSOR == 0):
                    dc_motor.stopConveyor()
                    current_step = Step.fourth_part_irsensor_post
            # 불량품 판정을 받았을 때 컨베이어벨트 정지 타이밍                                 
            else:
                print(pass_or_fail_eds)
                time.sleep(5)
                dc_motor.stopConveyor()
                current_step = Step.start





    ##########################################################################
        case Step.fourth_part_irsensor_post:
            print(Step.fourth_part_irsensor_post)

            if LIGHT_IR_SENSOR == 0:
                # 서버에서 적외선 센서 감지 여부 전송
                detect_reply = servercomm.confirmationObject(4, LIGHT_IR_SENSOR, "LIGHT_IR_SENSOR")
                # 답변 중 msg 변수에 "ok" 를 확인할 시
                if detect_reply == "ok":
                    current_step = Step.fourth_part_process_start

        case Step.fourth_part_process_start:  # 계산함수 시작조건 - 센서감지
            print(Step.fourth_part_process_start)
            start_reply = servercomm.euvLithographyStart()
            # 조도센서가 임무를 수행
            # 답변 중 msg 변수에 "ok" 를 확인할 시
            if start_reply == "ok":
                current_step = Step.fourth_part_process_sleep
            elif start_reply == "fail":
                current_step = Step.start
            time.sleep(5)  # time 모듈을 사용하도록 수정
        case Step.fourth_part_process_sleep:
            # 랜덤값 변수 대입 후 딜레이 (제조 시간 구현)
            print(Step.fourth_part_process_sleep)
            random_time = random.randint(4, 8)
            time.sleep(random_time)
            # 딜레이(제조)가 다 끝나면
            current_step = Step.fourth_part_sensor_measure_and_endpost
                    

        case Step.fourth_part_sensor_measure_and_endpost:
            print(Step.fourth_part_sensor_measure_and_endpost)
            # LED 불켬 코드 추가
            light_sensor.turn_on_led()
            time.sleep(0.5)
            # 조도센서값을 판단
            light_value = light_sensor.measure_light()
            print(light_value)
            # 조도센서 값을 서버에 전송
            end_light = servercomm.euvLithographyEnd(light_value)
            if end_light == "fail":
                pass_or_fail_grade = GuideMotorStep.reset
                pass_or_fail_euv = GuideMotorStep.fail
            else:
                if end_light == "left":
                    pass_or_fail_grade = GuideMotorStep.badGrade
                    pass_or_fail_euv = GuideMotorStep.good
                elif end_light == "right":
                    pass_or_fail_grade = GuideMotorStep.goodGrade
                    pass_or_fail_euv = GuideMotorStep.good
            # LED OFF
            light_sensor.turn_off_led()
            current_step = Step.move_servo
                

        case Step.move_servo:
            print(Step.move_servo)
            servo_euv_motor.doGuideMotor(pass_or_fail_euv)
            servo_grade_motor.doGuideMotor(pass_or_fail_grade)

            current_step = Step.go_rail_next_1
                

        case Step.go_rail_next_1:
            print(Step.go_rail_next_1)
            dc_motor.doConveyor()  # 모터를 구동시킴
            if LIGHT_IR_SENSOR == 1:
                servercomm.confirmationObject(4, LIGHT_IR_SENSOR, "LIGHT_IR_SENSOR")
                current_step = Step.stop_rail_1

        case Step.stop_rail_1:
            print(Step.stop_rail_1)
            if pass_or_fail_euv == GuideMotorStep.fail:
                time.sleep(5)  # time 모듈을 사용하도록 수정  # 5초 동안 대기
            else:
                time.sleep(5)  # time 모듈을 사용하도록 수정

            dc_motor.stopConveyor()  # 모터를 정지시킴
            current_step = Step.end_time

        case Step.end_time:
            print(Step.end_time)
            servercomm.confirmationObject(4, LIGHT_IR_SENSOR, "END_TIME")
            current_step = Step.start    
        
                