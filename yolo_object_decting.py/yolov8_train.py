from ultralytics import YOLO

DATA = "/Users/simseunghwan/Desktop/xyz/data.yaml"  # 위에서 수정한 yaml 경로

# 사전학습 가중치로 시작(가벼운 n 모델)
model = YOLO("yolov8n.pt")

model.train(
    data=DATA,
    epochs=100,          # 10은 조금 짧아요. 50~100 권장
    imgsz=640,           # 320 -> 640로 상향
    batch=-1,            # 자동 배치(메모리에 맞춰 조정). 부족하면 4~8로 수동
    device=-1,            # GPU 없으면 -1 또는 cpu
    workers=2,           # Mac에서 너무 높이면 느릴 수 있어 2 권장
    patience=20,         # 개선 없으면 조기 종료
    plots=True,          # 학습 곡선/혼동행렬 저장
    verbose=True,
    project="runs", name="detect_train_squirrel"  # 출력 폴더 이름 고정(선택)
)
