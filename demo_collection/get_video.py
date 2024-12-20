import numpy as np
import cv2

# Load the npy file
video_data = np.load('rgb_vid.npy')

# Get the shape of the video data
frames, height, width, channels = video_data.shape

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('rgb_vid.mp4', fourcc, 30.0, (width, height))

# Write each frame to the video file
for i in range(frames):
    frame = video_data[i]
    out.write(frame)

# Release the VideoWriter object
out.release()