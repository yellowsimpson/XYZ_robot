import cv2
from ultralytics import YOLO

# model = YOLO('yolov8s.pt')

model = YOLO('/Users/simseunghwan/Desktop/xyz/yolov8n.pt')

results = model('/Users/simseunghwan/Desktop/xyz/valid/images/-9-_jpeg.rf.b5a583c7887587869478503db2babfc5.jpg')

plots = results[0].plot()
cv2.imshow("plot", plots)
cv2.waitKey(0)
cv2.destroyAllWindows()