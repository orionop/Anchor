import cv2
import numpy as np
import rospy
from std_msgs.msg import Point

class TargetTracker:
    def __init__(self, camera_id=0):
        self.camera_id = camera_id
        self.cap = None
        self.target_publisher = rospy.Publisher('/target_position', Point, queue_size=10)
        self.rate = rospy.Rate(30)  # 30 Hz update rate
        
        # Initialize target detection parameters
        self.target_detector = cv2.createBackgroundSubtractorMOG2(history=500, detectShadows=False)
        self.kernel = np.ones((5,5), np.uint8)
        
        # Target tracking state
        self.target_found = False
        self.last_target_position = None
        self.tracking_threshold = 50  # pixels

    def initialize_camera(self):
        """Initialize the camera with error handling"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                raise Exception("Failed to open camera")
            return True
        except Exception as e:
            rospy.logerr(f"Camera initialization error: {str(e)}")
            return False

    def detect_target(self, frame):
        """Detect the target using motion detection and contour analysis"""
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Apply background subtraction
            fgmask = self.target_detector.apply(gray)
            
            # Noise removal
            fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, self.kernel)
            fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, self.kernel)
            
            # Find contours
            contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Calculate center point
                center_x = x + w//2
                center_y = y + h//2
                
                return (center_x, center_y), (x, y, w, h)
            
            return None, None
            
        except Exception as e:
            rospy.logerr(f"Target detection error: {str(e)}")
            return None, None

    def track_target(self):
        """Main tracking loop with error handling"""
        if not self.initialize_camera():
            return

        try:
            while not rospy.is_shutdown():
                ret, frame = self.cap.read()
                if not ret:
                    rospy.logerr("Failed to read frame from camera")
                    continue

                # Detect target
                target_pos, bbox = self.detect_target(frame)
                
                if target_pos is not None:
                    self.target_found = True
                    self.last_target_position = target_pos
                    
                    # Publish target position
                    target_msg = Point()
                    target_msg.x = target_pos[0]
                    target_msg.y = target_pos[1]
                    target_msg.z = 0  # Assuming 2D tracking
                    self.target_publisher.publish(target_msg)
                    
                    # Draw tracking visualization
                    if bbox:
                        x, y, w, h = bbox
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.circle(frame, target_pos, 5, (0, 0, 255), -1)
                
                # Display frame
                cv2.imshow('Target Tracking', frame)
                
                # Break loop on 'q' press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                self.rate.sleep()
                
        except Exception as e:
            rospy.logerr(f"Tracking error: {str(e)}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        rospy.init_node('target_tracker')
        tracker = TargetTracker()
        tracker.track_target()
    except rospy.ROSInterruptException:
        pass
