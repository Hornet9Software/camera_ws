import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import math








class oflow(Node):

    def __init__(self):
        super().__init__("oflow")
        self.pub_oflow = self.create_publisher(CompressedImage, "oflow/compressed", 10)
        #self.pub_vel = self.create_publisher(CompressedImage, "oflow/compressed", 10)
        self.track_len = 2
        self.detect_interval = 4
        self.tracks = []
        self.alpha = 0.5
        self.frame_idx = 0
        

        self.front_image_feed = self.create_subscription(
            CompressedImage,
            "/Hornet/Cam/Floor/image_rect_color/compressed",
            self.image_feed_callback,
            10)
        self.bridge = CvBridge()

    def image_feed_callback(self, msg):

        # bridge is from cv_bridge (for converting ROS image/CompressedImage to opencv images)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        vel, vis = self.optical_flow(cv_img)
        print(vel)
        final_msg = self.bridge.cv2_to_compressed_imgmsg(vis) 
        # Convert final image (cv_img) and publish

        self.pub_oflow.publish(final_msg)

    def optical_flow(self, frame):
        cnt = 0
        # Lucas-Kanade parameters
        lk_params = dict(winSize=(15, 15),
                        maxLevel=2,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        feature_params = dict(maxCorners=700,
                            qualityLevel=0.3,
                            minDistance=7,# previously 7
                            blockSize=7) # previously 7

        # Constants
        fps = 30
        px2m1 = 1
        ms2kmh = 3.6

        cal_polygon = np.array([[0, 0], [0, 2500], [1080, 2500], [1080, 0]])
        view_polygon = np.array([[0, 0], [0, 2500], [1080, 2500], [1080, 0]])
        prv1 = 0
        prn1 = 0
        ptn1 = 0  
        cal_mask = np.zeros_like(frame[:, :, 0])
        view_mask = np.zeros_like(frame[:, :, 0])

        polygon1 = np.array([[0, 0], [0, 2500], [1080, 2500], [1080, 0]])
        cv2.fillConvexPoly(cal_mask, cal_polygon, 1)
        cv2.fillConvexPoly(view_mask, view_polygon, 1)
        vis = frame.copy()
        cmask = frame.copy()
        mm1 = 0
        v1 = 0

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.bitwise_and(frame_gray, frame_gray, mask=cal_mask)

        vis = cv2.bitwise_and(vis, vis, mask=view_mask)
        # cv2.putText(vis, str(prv1) ,(30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # cv2.putText(vis, str(prn1), (900, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if len(self.tracks) > 0:
            img0, img1 = self.prev_gray, frame_gray
            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
            p1, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
            p0r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
            d = abs(p0-p0r).reshape(-1, 2).max(-1)
            good = d < 1
            new_tracks = []
            for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > self.track_len:
                    del tr[0]
                new_tracks.append(tr)
                cv2.circle(vis, (int(x), int(y)), 3, (0, 255, 0), -1)
            self.tracks = new_tracks

            ptn1, ptn2, ptn3, ptn4, ptn5 = 0, 0, 0, 0, 0
            import time
            start = time.time()  
            for idx, tr in enumerate(self.tracks):
                result_polygon1 = cv2.pointPolygonTest(polygon1, tr[0],True)
                if result_polygon1 > 0:
                    ptn1 += 1
                    dif1 = tuple(map(lambda i, j: i - j, tr[0], tr[1]))
                    mm1 += math.sqrt(dif1[0]*dif1[0] + dif1[1]*dif1[1])
                    mmm1 = mm1/ptn1
                    v1 = mmm1*px2m1*fps*ms2kmh*100000/3600/100
            cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 0, 255))

        prn1 = ptn1

        if self.frame_idx % self.detect_interval == 0:
            if ptn1 > 10:
                
                cv2.putText(vis, str(prv1) ,(30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                cv2.putText(vis, str(prn1), (900, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

            # Speed writing part
            prv1 = v1
            mask = np.zeros_like(frame_gray)
            mask[:] = 255
            for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                cv2.circle(mask, (int(x), int(y)), 4, 0, -1)
            p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    self.tracks.append([(x, y)])


        self.frame_idx += 1
        self.prev_gray = frame_gray
        cv2.addWeighted(cmask, self.alpha, vis, 1 - self.alpha, 0, vis)
        cnt += 1
        # print(cnt)

        return v1, vis

def main(args=None):
    rclpy.init(args=args)
    node = oflow()
    rclpy.spin(node)

    # Below lines are not strictly necessary
    node.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()