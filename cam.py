import picamera
import os
record_time = 120
log = open("cam_log.txt","w+")
log.write("Log file Opened")
camera = picamera.PiCamera(stereo_mode = 'side-by-side')
log.write("Cameras initialized...\n")
camera.resolution = (1280, 720)
camera.start_recording('/boot/my_video.h264')
camera.wait_recording(record_time)
log.write("Recording complete...\n")
camera.stop_recording()
video = os.path.getsize('/boot/my_video.h264')
log.write(str(video) + "bytes\n")
log.close()
