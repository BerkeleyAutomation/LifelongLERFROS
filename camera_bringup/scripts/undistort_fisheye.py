import cv2
import numpy as np
import sys

DIM=(1024,576)
K=np.array([[434.0546822371517, 0.0, 527.2416179357791], [0.0, 436.315008776941, 315.44148105331647], [0.0, 0.0, 1.0]])
D=np.array([[-0.040568173151572746], [0.06473101757464744], [-0.10019273635880038], [0.0461702340651986]])

def undistort(img_path):    
    img = cv2.imread(img_path)
    h,w = img.shape[:2]    
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)    
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)