# Detect wall corner
# camera view vision


def calculate_object_distance(pitch, detectedRatio):

	# below function is calculated by dataset with r = 0.997414693
	# ratio = 0.538431 - 0.02380893333 * pitch + 4.80444444 * 0.0001 * pitch * pitch
	ratio = 0.519 - 0.02380893333 * pitch + 4.80444444 * 0.0001 * pitch * pitch

	diff = detectedRatio - ratio

	if diff <= 0:
		return 0

	# this new ratio is from the dataset with pitch = 0
	newRatio = 0.513 + diff
	distance = 349 - 1360 * newRatio + 1330 * newRatio * newRatio

	return distance

datasets = []
for pitch in ["-7_5", "0", "7_5", "15"]: # -15
	for distance in ["1", "1_5", "2", "2_5", "3"]: # 3_5
		datasets.append({"pitch": pitch, "distance": distance})

import cv2

# img = cv2.imread('source/-15x3.jpg')

classNames = []
classFile = 'coco.names'
with open(classFile, 'r') as f:
	classNames = f.read().rstrip('\n').split('\n')

configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

for dataset in datasets:

	pitch = dataset["pitch"]
	distance = dataset["distance"]
	filename = "source/" + pitch + "x" + distance + ".jpg"

	img = cv2.imread(filename)
	height, width, channels = img.shape
	# print(height, width, channels)

	classIds, confidences, bbox = net.detect(img, confThreshold = 0.55)
	# print(classIds, confidences, bbox)

	plantClassIds = []
	plantConfidences = []
	plantBBox = []
	for i in range(len(classIds)):
		classId = classIds[i]
		confidence = confidences[i]
		box = bbox[i]
		if classNames[classId - 1].upper() == "POTTED PLANT":
			plantClassIds.append(classId)
			plantConfidences.append(confidence)
			plantBBox.append(box)

	maxConf = max(plantConfidences)
	for i in range(len(plantClassIds)):
		classId = plantClassIds[i]
		confidence = plantConfidences[i]
		box = plantBBox[i]

		if confidence == maxConf:

			pitch_num = float(pitch.replace("_", "."))
			distance_num = float(distance.replace("_", "."))

			realDetectedRatio = round((height - box[1] - box[3]) / height, 4)

			if pitch_num == -7.5 and distance_num == 1:
				box[3] = box[3] - 12
			elif pitch_num == -7.5 and distance_num == 1.5:
				box[3] = box[3] - 25
			elif pitch_num == -7.5 and distance_num == 2:
				box[3] = box[3] + 5
			elif pitch_num == 0 and distance_num == 2:
				box[3] = box[3] - 13
			elif pitch_num == 0 and distance_num == 3:
				box[3] = box[3] + 18
			elif pitch_num == 7.5 and distance_num == 1:
				box[3] = box[3] - 3
			elif pitch_num == 7.5 and distance_num == 1.5:
				box[3] = box[3] + 23
			elif pitch_num == 7.5 and distance_num == 2.5:
				box[3] = box[3] + 5
			elif pitch_num == 7.5 and distance_num == 3:
				box[3] = box[3] + 22
			elif pitch_num == 15 and distance_num == 1.5:
				box[3] = box[3] + 9
			elif pitch_num == 15 and distance_num == 2:
				box[3] = box[3] + 2

			x, y, w, h = box[0], box[1], box[2], box[3]

			cv2.rectangle(img, box, color = (0, 255, 0), thickness = 2)
			label = classNames[classId - 1].upper() + " " + filename + " " + str(confidence)
			cv2.putText(img, label, (x + 10, y + 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

			detectedRatio = round((height - y - h) / height, 4)
			print(detectedRatio, pitch_num, distance_num)
			print("correct distance", distance_num)
			print("distance", calculate_object_distance(pitch_num, realDetectedRatio))

	# cv2.imshow("Output", img)
	# cv2.waitKey(0)