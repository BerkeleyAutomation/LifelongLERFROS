#!/usr/bin/env python3
import cv2
import os

class ChessboardDetector:
    def __init__(self):
        self.image_count = 0
        self.save_folder = 'chessboard_images'

        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)

    def process_video(self):
        cap = cv2.VideoCapture(0)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            cv_image = frame
            cv_image_original = cv_image.copy()

            success, corners = self.detect_chessboard(cv_image)

            if success:
                cv2.drawChessboardCorners(cv_image, (8, 5), corners, success)
                self.save_image(cv_image_original)

            # cv2.imshow('Chessboard Detection', cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def detect_chessboard(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (8, 5), None)
        return ret, corners

    def save_image(self, image):
        self.image_count += 1
        image_filename = f'{self.save_folder}/image_{self.image_count:04d}.png'
        cv2.imwrite(image_filename, image)
        print(f"Chessboard image saved: {image_filename}")

def main():
    detector = ChessboardDetector()
    detector.process_video()

if __name__ == '__main__':
    main()
