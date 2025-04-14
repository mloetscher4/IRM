import cv2
import numpy as np
import os

def resize_to_aspect_ratio(image, target_ratio, final_size=None):
    target_w, target_h = target_ratio
    assert isinstance(image, np.ndarray), "Variable is not a NumPy array"
    img_h, img_w = image.shape[:2]
    input_ratio = img_w / img_h
    target_ratio_val = target_w / target_h

    if input_ratio > target_ratio_val:
        new_width = int(img_h * target_ratio_val)
        offset = (img_w - new_width) // 2
        cropped = image[:, offset:offset + new_width]
    else:
        new_height = int(img_w / target_ratio_val)
        offset = (img_h - new_height) // 2
        cropped = image[offset:offset + new_height, :]

    if final_size:
        return cv2.resize(cropped, final_size, interpolation=cv2.INTER_AREA)
    else:
        print(np.shape(cropped))
        return cropped

def extractImages(pathIn, pathOut):
    count = 0
    cut_t = 100  # frames to cut from start and end
    number_img = 30
    target_ratio = (1150, 750)

    vidcap = cv2.VideoCapture(pathIn)
    assert vidcap.isOpened(), "Failed to open video file"

    frame_count = int(vidcap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Total frames in video: {frame_count}")
    
    # Compute safe range and step
    start_frame = cut_t
    end_frame = frame_count - cut_t
    step = (end_frame - start_frame) // number_img

    for i in range(number_img):
        frame_index = start_frame + i * step
        vidcap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        success, image = vidcap.read()
        print(f"Reading frame {frame_index}: success = {success}")
        assert success, f"Failed to read frame at index {frame_index}"
        image_cropped = resize_to_aspect_ratio(image, target_ratio)
        filename = os.path.join(pathOut, f"frame{count}.jpg")
        cv2.imwrite(filename, image_cropped)
        count += 1

    vidcap.release()
    print("Done extracting images.")


if __name__=="__main__":
    pathIn = "/home/irm/Desktop/irm/lab05/IRM/Lab05_Ball_Balancing_1/skeleton/camera_calibration/Aufzeichnung_2025-04-07_140538.mp4"
    pathOut = "/home/irm/Desktop/irm/lab05/IRM/Lab05_Ball_Balancing_1/skeleton/camera_calibration/"
    extractImages(pathIn, pathOut)