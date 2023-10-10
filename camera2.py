import cv2


def gstreamer_pipeline():
    return (
        "v4l2src device=/dev/video0 ! "
        "video/x-raw(memory:NVMM), width=(int)640, height=(int)48-, format=(string)UYVY, framerate=30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=True sync=False"
    )


def main():
    # Create an OpenCV VideoCapture object
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Failed to open the camera.")
        exit()

    # Loop to continuously get each frame and display it
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab the frame.")
            break

        cv2.imshow('Camera Feed', frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
