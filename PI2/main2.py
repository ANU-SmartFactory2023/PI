import random
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import time



# 모듈 또는 파일 불러오기
from enum import Enum
from common.server_communication import ServerComm
from common.irSensor import InfraredSensor
from common.ultrasonicSensor import UltrasonicSensor
from common.motor import Motor, GuideMotorStep
from measure import Measure


class Step( Enum ) :    #각 스텝별 이름, 동사형으로 지을것, 무엇을 하는 스텝인지 알 수 있는 네이밍
    start=0    
    process_start = 10   
    sonic_part_sensor_check = 100    
    measure_start = 300   
    stop_rail = 500
    calculated_values_send = 600
    servo_motor_drive = 700
    process_check = 750
    go_rail_next = 800

# 기본설정
currnet_step = Step.start   
running = True  
pass_or_fail = ''   #서버에서 주는 담아주는 값이 string 형태

# 값계산 함수 기본변수
min = 2
max = 7
measure = Measure( min, max )

# 핀번호 지정
# DC모터 핀
DC_enable_pin = 1
DC_input1_pin = 2
DC_input2_pin = 3
# 서보모터 핀
SERVO_PIN = 16  
# 적외선 핀
SONIC_IR_SENSOR_PIN_NO1 = 19
SONIC_IR_SENSOR_PIN_NO2 = 20
RELAY_IR_SENSOR_PIN = 21
# 소닉센서 핀
UltraSonic_trig_pin_NO = 13
UltraSonic_echo_pin_NO = 12


# 함수
sonic_ir_sensor_1 = InfraredSensor( SONIC_IR_SENSOR_PIN_NO1 )
sonic_ir_sensor_2 = InfraredSensor( SONIC_IR_SENSOR_PIN_NO2 )
relay_ir_sensor = InfraredSensor( SONIC_IR_SENSOR_PIN_NO2 )
sonic_module = UltrasonicSensor( UltraSonic_trig_pin_NO, UltraSonic_echo_pin_NO, ) #소닉센서핀
servercomm = ServerComm()
dc_motor = Motor().dc_init( DC_enable_pin, DC_input1_pin, DC_input2_pin  ) 
servo_motor = Motor().servo_init( SERVO_PIN )



while running:
    print( "running : " + str( running ) )# 디버깅확인용
    time.sleep( 0.1 )
    SONIC_IR_SENSOR_NO1 = sonic_ir_sensor_1.measure_ir()
    SONIC_IR_SENSOR_NO2 = sonic_ir_sensor_2.measure_ir()
    RELAY_IR_SENSOR = relay_ir_sensor.measure_ir()
    match currnet_step :

        case Step.start: 
            print( Step.start )
            dc_motor.doGuideMotor( GuideMotorStep.stop )
            #시작하기전에 검사할것들: 통신확인여부, 모터정렬, 센서 검수
            currnet_step = Step.sonic_part_sensor_check #다음스텝으로 이동

        case Step.sonic_part_sensor_check:  #1번 초음파 센서에 감지
            print( Step.sonic_part_sensor_check )
            if( SONIC_IR_SENSOR_NO1 == 1):
                # 감지상태
                # 서버에게 센서 감지상태를 포스트로 전달한다.
                servercomm.confirmationObject( 2, SONIC_IR_SENSOR_NO1,'SONIC_IR_SENSOR_NO1' )
                servercomm.etchingStart( True )
                currnet_step = Step.measure_start

        case Step.measure_start:
            print( Step.measure_start )
            value = sonic_module.measure_distance ()  # 초음파센서값
            measure.add( value ) # measure에 값 삽입
           
            if( SONIC_IR_SENSOR_NO1 == 0):     #서버에 부하가 생기면 스텝으로 따로 빼야함 
                servercomm.confirmationObject( 2, SONIC_IR_SENSOR_NO1,'SONIC_IR_SENSOR_NO1' )    

            if(SONIC_IR_SENSOR_NO2 == 1 ): #2번 적외선센서 도달                
                servercomm.confirmationObject( 2 , SONIC_IR_SENSOR_NO2,'SONIC_IR_SENSOR_NO2' )
                currnet_step = Step.stop_rail

        case Step.stop_rail:
            print( Step.stop_rail )
            result = dc_motor.stopConveyor#DC모터 정지            
            currnet_step = Step.calculated_values_send

        case Step.calculated_values_send:
            resutl = measure.getAverage()     # 값계산
            pass_or_fail = servercomm.etchingEnd( result )  #서버에 값송신
            currnet_step = Step.servo_motor_drive
                
        case Step.servo_motor_drive:
            motor_step = GuideMotorStep.stop    #기본 stop
            if( pass_or_fail == 'fail'):                 #서버에서 받은 불량기준
                motor_step = GuideMotorStep.fail
            else :
                motor_step = GuideMotorStep.good

            servo_motor.doGuideMotor( motor_step )    #위에 따라서 모터구동 
            currnet_step = Step.process_check

        case Step.process_check:    #불량여부에 따라서 프로세스 수정
            result = dc_motor.doConveyor#DC모터 구동 

            if(pass_or_fail == 'False'):
                time.sleep(5)       
                dc_motor.stopConveyor()     
                currnet_step = Step.start
            else :
                if(SONIC_IR_SENSOR_NO2 == 0 ): #2번 적외선센서 off                
                    servercomm.confirmationObject( 2 , SONIC_IR_SENSOR_NO2,'SONIC_IR_SENSOR_NO2' )
                currnet_step = Step.go_rail_next
        
        case Step.go_rail_next:
            print( Step.go_rail_next )
            if( RELAY_IR_SENSOR == 1):
                dc_motor.stopConveyor()         
                currnet_step = Step.start


        
        
        