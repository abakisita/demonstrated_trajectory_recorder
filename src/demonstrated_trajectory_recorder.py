import cv2
import yaml
import numpy as np
import rospy
from cv2 import aruco
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, PoseStamped
import std_msgs
import tf

class record_trajectories():

    def __init__(self):
        
        rospy.init_node('demonstration_recorder')
        rospy.Rate(150)
        self.aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_6X6_1000 )

        #Provide length of the marker's side
        markerLength = 3.7  # Here, measurement unit is centimetre.

        # Provide separation between markers
        markerSeparation = 0.48   # Here, measurement unit is centimetre.

        # create arUco board
        self.board = aruco.GridBoard_create(4, 5, markerLength, markerSeparation, self.aruco_dict)

        '''uncomment following block to draw and show the board'''
        #img = board.draw((864,1080))
        #cv2.imshow("aruco", img)

        self.arucoParams = aruco.DetectorParameters_create()  
        self.tf = tf.TransformListener()
        #check if camera is working or not
        self.camera = cv2.VideoCapture(1)
        ret, img = self.camera.read()
        cv2.imshow("debug", img) 
        with open('../data/'+'camera_calibration.yaml') as f:
            loadeddict = yaml.load(f)
        mtx = loadeddict.get('camera_matrix')
        dist = loadeddict.get('dist_coeff')
        self.mtx = np.array(mtx)
        self.dist = np.array(dist)

        img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        h,  w = img_gray.shape[:2]
        self.newcameramtx, self.roi=cv2.getOptimalNewCameraMatrix(self.mtx, self.dist,(w,h),1,(w,h))
        self.path_publisher = rospy.Publisher('/recorded_path', Path, queue_size=1)
        rospy.Subscriber('/demonstration_recorder_event_in', std_msgs.msg.String, self.event_in_cb)

        self.event_in = None
        self.state = 'INIT'

    def event_in_cb(self, msg):
        print "event recieved", msg.data
        self.event_in = msg.data

    def record(self):
        pose_r = []
        pose_t = []
        first_run = True
        while True:    
            ret, img = self.camera.read()
            img_aruco = img
            if first_run == True:
                first_run = False
                for i in range(10):
                    im_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
            im_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
            h,  w = im_gray.shape[:2]
            dst = cv2.undistort(im_gray, self.mtx, self.dist, None, self.newcameramtx)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(dst, self.aruco_dict, parameters=self.arucoParams)
            if corners == None:
                print ("pass")
            else:
                ret, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, self.newcameramtx, self.dist) # For a board
                if ret != 0:
                    pose_r.append(rvec * 0.01)
                    pose_t.append(tvec * 0.01)
                    img_aruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
                    img_aruco = aruco.drawAxis(img_aruco, self.newcameramtx, self.dist, rvec, tvec, 10)    
                    
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    path = Path()
                    path.header.frame_id = 'base_link'
                    linear = []
                    rotational = []
                    for itr in range(len(pose_t)):
                        r =  pose_r[itr]
                        quaternion = tf.transformations.quaternion_from_euler(r[0, 0], r[1, 0], r[2, 0])
                        t = pose_t[itr]
                        pose_temp = Pose()
                        pose_temp.position.x = t[0, 0]
                        pose_temp.position.y = t[1, 0]
                        pose_temp.position.z = t[2, 0]
                        pose_temp.orientation.x = quaternion[0]
                        pose_temp.orientation.y = quaternion[1]
                        pose_temp.orientation.z = quaternion[2]
                        pose_temp.orientation.w = quaternion[3]

                        pose_stmpd = PoseStamped()

                        pose_stmpd.header.frame_id = "arm_cam3d_camera"
                        pose_stmpd.pose = pose_temp
                        while True:
                            try :
                                pose_stmpd = self.tf.transformPose('/base_link', pose_stmpd)
                                break
                            except:
                                print "looping"
                                continue

                        path.poses.append(pose_stmpd)

                        linear.append([pose_stmpd.pose.position.x, pose_stmpd.pose.position.y,\
                                pose_stmpd.pose.position.z])
                        euler = tf.transformations.euler_from_quaternion((pose_stmpd.pose.orientation.x, \
                                pose_stmpd.pose.orientation.y , pose_stmpd.pose.orientation.z, pose_stmpd.pose.orientation.w))
                        rotational.append(euler)

                    
                    self.path_publisher.publish(path)
                    trajectory_file_name = raw_input('enter trajectory_file_name')
                    data = {'linear_trajectory': np.asarray(linear).tolist(), \
                            'rotational_trajectory': np.asarray(rotational).tolist()}
                    with open("../data/"+trajectory_file_name, "w") as f:
                        yaml.dump(data, f) 
                    break
                    cv2.waitKey(100)

                cv2.imshow("World co-ordinate frame axes", img_aruco)
        cv2.destroyAllWindows()        
        return 'INIT'

    def init(self):
        
        print "Node initiated"
        self.event_in = None
        return 'IDLE'

    def running(self):

        while True:
            if self.state == 'INIT' :
                self.state = self.init()
            elif self.state == 'RUNNING':
                self.state = self.record()
            elif self.state == 'IDLE':
                self.state = self.idle()

    def idle(self):
        
        if self.event_in == 'e_start':
            return 'RUNNING'
        else :
            a = raw_input("input something to start again")
            self.event_in = 'e_start'
            return 'IDLE'



if __name__=="__main__":
    
    obj = record_trajectories()
    obj.running()

