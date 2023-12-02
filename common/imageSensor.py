import cv2
import numpy as np

class ImageCV:
    def __init__(self):
         # 웹캠 열기
         self.cap = cv2.VideoCapture(0)

    # def __del__(self):
    #     self.cap.release()
    #     cv2.destroyAllWindows()

    def count_black_pixels(self):
       
        image = self.cap.read()[1]

        # 이미지를 흑백으로 변환
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 가우시안 블러를 사용하여 입력 영상의 잡음 제거
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

        # 이진화를 위한 임계값 설정 (여기서는 40을 기준으로 이진화)
        _, binary_image = cv2.threshold(blurred_image, 40, 255, cv2.THRESH_BINARY)

        # 검은색 픽셀 수 계산
        black_pixel_count = np.sum(binary_image == 0)

        return black_pixel_count

#     def measure_image_data(self):
#         # 웹캠에서 프레임 읽기
#         image = self.cap.read()[1]
#         # 이미지 데이터 측정
#         black_pixel_count = self.count_black_pixels(image)

#         return black_pixel_count
    
#     def run_webcam_binarization(self, threshold=10000):
#         while True:
#             # 웹캠에서 프레임 읽기
#             frame = self.cap.read()[1]
#             if frame is None:
#                 frame = 0  # 프레임을 읽지 못한 경우 0으로 간주

#             cv2.imshow('Webcam', self.resize_frame(frame, width=640, height=480))

#             # 검은색 픽셀 수 측정
#             final_black_pixel_count = self.measure_image_data()

#             # black_pixel_count가 threshold 이상이면 종료
#             if final_black_pixel_count is not None and final_black_pixel_count >= threshold:
#                 print("Poor")
#                 break

#         # # 웹캠 해제 및 창 닫기
#         # self.cap.release()  # __del에서 중복되는 부분

#         # 모든 창 닫기
#         cv2.destroyAllWindows()

# if __name__ == "__main__":
#     # ImageCV 클래스 인스턴스 생성
#     image_processor = ImageCV()
    
#     # 웹캠 이진화 실행
#     image_processor.run_webcam_binarization()