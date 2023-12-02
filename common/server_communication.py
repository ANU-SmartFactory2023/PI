#server_communication.py
import http.client
import json

#적외선 센서와 2차 센서 역할이 다름으로 모델 추가 및 변경
from common.models import SensorModel
from common.models import ProcessModel

# 클래스의 메서드에서 첫 번째 매개변수로 self를 사용하는 것은 파이썬의 규칙 중 하나입니다. 
# self를 사용하여 클래스의 인스턴스 변수 및 메서드에 접근할 수 있다.
class ServerComm :
    conn:http.client.HTTPConnection
    headers = {"Content-type": "application/json", "Accept": "*/*"}
    fail_msg = '{"msg":"fail","statusCode":"500"}' 
    
    #__init__ 설정 메서드에 서버 ip주소 및 포트번호 설정
    def __init__( self ) :
        self.conn = http.client.HTTPConnection( '192.168.41.238', 5000 ) # 서버 ip, 포트
        self.conn.timeout = 3

    # HTTP 통신 Sensor Post 정의
    def sensorRequestPost( self, url, s:SensorModel ) :
        result = ""
        try:
            self.conn.request( 'POST', url, json.dumps( s.__dict__ ), self.headers )
            result = self.conn.getresponse().read().decode()
        except:
            result = self.fail_msg
               
        json_object = json.loads( result )  
        # json 안 답변 분리 후 변수 저장 가능
        msg = json_object[ 'msg' ] 
        statusCode = json_object[ 'statusCode' ]   

        print("Server response:", msg)  # 받은 응답 출력

        return msg 

    # HTTP 통신 Process Post 정의
    def ProcessRequestPost( self, url, p:ProcessModel ) :
        result = ""
        try:
            # p 클래스 변수들을 딕셔너리 형태로 변환 후 전송
            self.conn.request( 'POST', url, json.dumps( p.__dict__ ), self.headers )
            # getresponse()를 호출하면 http.client.HTTPResponse 객체 반환
            # read() 메서드를 호출하여 응답 데이터를 읽고
            # decode()를 사용하여 해당 데이터를 문자열로 디코딩
            result = self.conn.getresponse().read().decode()
        except:
            result = self.fail_msg

        # json.loads 함수를 사용하여 이를 파이썬 객체로 변환한다.
        json_object = json.loads( result )  
        
        # json 안 답변 분리 후 변수 저장 가능
        msg = json_object[ 'msg' ] 
        statusCode = json_object[ 'statusCode' ] 

        print("Server response:", msg)  # 받은 응답 출력
        
        # print("Server response:", json_object)  # 받은 응답 출력

        # 서버에서 json 파일 답변 여부를 확인하는 예외 처리
        # try:
        #     json_object = json.loads(result)
        #     return json_object
        # except json.decoder.JSONDecodeError as e:
        #     print(f"Error decoding JSON: {e}")
        #     return None

        return msg

    # HTTP 통신 Get 정의
    def requestGet( self, url ) :
        result = ""
        try:
            self.conn.request( 'GET', url  )
            result = self.conn.getresponse().read().decode()
        except:
            result = self.fail_msg
            
        json_object = json.loads( result )

        return json_object
    
    # 공정 시작 전 제품 도착 여부 전송 (Get)
    def ready(self) :
        json_object = self.requestGet( '/pi/sensor/0' )

        msg = json_object[ 'msg' ]
        if msg == 'ok':
            return True
        else:
            return False
        
    # 2차 공정 양품 여부 파악 (Get)
    def check_second_process(self) :
        json_object = self.requestGet( '/pi/process/2' )

        msg = json_object[ 'msg' ]
        if msg == 'pass':
            return True
        else:
            return False

    # 1~4 차 제조 공정 전 적외선 센서를 사용해 제품 도착 여부 전송 (Post)
    def confirmationObject( self, idx, on_off ) :
        s = SensorModel()
        
        # 적외선 센서는 한가지 종류만 있어 "detect" 로 고정
        s.sensorName = "detect"
        # 서버에서 on과 off에 따라 로직이 달라짐
        if(on_off == 1):
            s.sensorState = "on"
        else:
            s.sensorState = "off"

        res = self.sensorRequestPost( f'/pi/sensor/{idx}', s )

        return res
    
    # 각 공정마다 인수를 넣지 않고 간단하게 호출할 수 있도록
    # 공정마다 매개변수를 통신 클래스에서 먼저 정의하는 방법

    # 포토 공정 시작 시간 전송
    def photoStart( self ):
        return self.__checkProcess( 1, "start", "photo", "0")
    # 포토 공정 종료 타이밍과 센서값 전송
    def photoEnd( self, processValue):
        return self.__checkProcess( 1, "end", "photo", processValue)
    
    # 식각 공정 시작
    def etchingStart( self ):
        return self.__checkProcess( 2, "start", "etching", "0")
    # 식각 공정 종료 
    def etchingEnd( self, processValue):
        return self.__checkProcess( 2, "end", "etching", processValue)

    # 이온 주입 공정 시작 
    def ionlmplantationStart( self ):
        return self.__checkProcess( 3, "start", "ionlmplantation", "0")
    # 이온 주입 공정 종료
    def ionlmplantationEnd( self, processValue):
        return self.__checkProcess( 3, "end", "ionlmplantation", processValue)

    # 후공정 시작 
    def metalWiringStart( self ):
        return self.__checkProcess( 4, "start", "metalWiring", "0")
    # 후공정 종료
    def metalWiringEnd( self, processValue):
        return self.__checkProcess( 4, "end", "metalWiring", processValue) 
    

    # 1~4 차 제조 공정 후 불량품 구분을 위한 센서값 전송 (Post)
    def __checkProcess( self, idx, processCmd, processName, processValue):
        p = ProcessModel()
        p.processCmd = processCmd
        p.processName = processName
        p.processValue = processValue

        res = self.ProcessRequestPost( f'/pi/process/{idx}', p )

        return res