from ultralytics import YOLO
import cv2, matplotlib.pyplot as plt

WEIGHTS = "/Users/simseunghwan/Desktop/xyz/runs/detect_train_squirrel/weights/best.pt"
IMG = "/Users/simseunghwan/Desktop/xyz/valid/images/-9-_jpeg.rf.b5a583c7887587869478503db2babfc5.jpg"

model = YOLO(WEIGHTS)

# (A) 검증 지표(mAP) 확인
model.val(data="/Users/simseunghwan/Desktop/xyz/data.yaml", imgsz=640)

# (B) 단일 이미지 시각화
res = model.predict(source=IMG, imgsz=640, conf=0.25, save=True, show=False)
annot = res[0].plot()
plt.imshow(cv2.cvtColor(annot, cv2.COLOR_BGR2RGB)); plt.axis("off"); plt.show()
