import cv2
import numpy as np

class ImageCV:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def count_white_pixels(self, image):
        # 이미지를 흑백으로 변환
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 가우시안 블러를 사용하여 입력 영상의 잡음 제거
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

        # 이진화를 위한 임계값 설정 (여기서는 40을 기준으로 이진화)
        _, binary_image = cv2.threshold(blurred_image, 40, 255, cv2.THRESH_BINARY)

        # 흰색 픽셀 수 계산
        white_pixel_count = np.sum(binary_image == 255)

        return white_pixel_count