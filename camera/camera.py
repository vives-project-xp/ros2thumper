import cv2

ds_factor=1 #change this value if you want to scale the video stream

class VideoCamera(object):
    def __init__(self):
        self.video = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
    
    def __del__(self):
        self.video.release()
    
    def get_frame(self):
        success, image = self.video.read()
        image=cv2.resize(image,None,fx=ds_factor,fy=ds_factor,interpolation=cv2.INTER_AREA)
        image=cv2.rotate(image, cv2.ROTATE_180)
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        ret, jpeg = cv2.imencode('.jpg', image)
        return jpeg.tobytes()