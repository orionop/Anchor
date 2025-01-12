import cv2
import numpy as np

class TargetTracker:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)  # using the default camera
        self.target_cascade = cv2.CascadeClassifier('target_cascade.xml')  # XML file for target detection

    def track_target(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            targets = self.target_cascade.detectMultiScale(gray, 1.3, 5)

            for (x, y, w, h) in targets:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

            cv2.imshow('Target Tracking', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

# Example usage:
if __name__ == "__main__":
    tracker = TargetTracker()
    tracker.track_target()
