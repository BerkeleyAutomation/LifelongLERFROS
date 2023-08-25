import cv2 as cv
import numpy as np

img = cv.imread('/home/benchturtle/lifelong_lerf_ws/output_images/test.jpg')
K = np.array([[1.01601115e+03, 0.00000000e+00, 5.14346488e+02],
 [0.00000000e+00, 2.03338996e+03, 2.69419609e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
D = np.array([[-0.40653068, -1.49320852,  0.09888003,  0.01690802 , 3.32428058]])
h, w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(K, D, (w,h), 1, (w,h))

# undistort
dst = cv.undistort(img, K, D, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imshow("undistorted", dst)
cv.waitKey(0)
cv.destroyAllWindows()