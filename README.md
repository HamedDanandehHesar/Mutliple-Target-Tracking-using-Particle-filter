# Multi-Object Tracking using Particle Filter in MATLAB

This project implements a **multi-object tracking system based on Particle Filters** in MATLAB.  
The algorithm detects objects in a video, tracks them across frames, and records their trajectories and positions.

The tracker uses:
- Particle Filtering for state estimation
- Hungarian (Munkres) algorithm for data association
- Simple image-based object detection
- Trajectory visualization and CSV logging

The output includes **tracked trajectories, annotated frames, and CSV trajectory data**.

---

# Features

- Multi-object tracking
- Particle filter state estimation
- Constant velocity motion model
- Hungarian (Munkres) assignment for track–detection matching
- Automatic track creation and deletion
- Trajectory visualization
- Particle visualization
- Export trajectory data to CSV
- Save processed frames as images

---

# Algorithm Overview

The system performs the following steps for each frame of the video:

1. **Frame Acquisition**
   - Read the next frame from the video.

2. **Object Detection**
   - Convert frame to grayscale
   - Apply binary thresholding
   - Extract object centroids using `regionprops`

3. **Particle Prediction**
   - Predict particle states using a **constant velocity motion model**.

4. **Data Association**
   - Compute cost matrix between predicted track positions and detections.
   - Apply the **Hungarian (Munkres) algorithm** to find optimal assignments.

5. **Particle Weight Update**
   - Update particle weights based on the distance between particles and detected objects.

6. **Resampling**
   - Apply **systematic resampling** when particle degeneracy occurs.

7. **Track Management**
   - Create new tracks for unmatched detections.
   - Increase strike count for missing detections.
   - Remove tracks after several missed frames.

8. **Trajectory Recording**
   - Store trajectories in memory and export them to a CSV file.

9. **Visualization**
   - Draw particles, trajectories, track IDs, and detections on the video frame.

---

# Input

The program requires:

- A video file (`.avi`)
- An output folder to save results

When running the script, MATLAB will ask the user to select:

1. The **video file**
2. The **output folder**

---

# Output

The program generates:

### 1. Annotated Frames
Each processed frame is saved as an image:

```
output_folder/
    1.jpg
    2.jpg
    3.jpg
    ...
```

These images contain:
- detected objects
- particle clouds
- object trajectories
- track IDs

### 2. CSV File

Trajectory data is saved as:

```
Particle_filter_veloc_trajectories.csv
```

CSV format:

```
Frame,Track_ID,Class,X,Y
```

Example:

```
1,0,target,215,340
1,1,target,510,120
2,0,target,220,338
```

---

# Parameters

Key parameters used in the tracker:

- `N_PARTICLES`  
  Number of particles per object

- `euclidean_dist_thresh`  
  Maximum distance allowed for track-detection assignment

- `max_track_strikes`  
  Maximum number of frames a track can miss before removal

- `process_noise`  
  Motion model noise

- `sensor_noise`  
  Measurement noise

- `trajectory_length`  
  Number of past points shown in trajectory visualization

These parameters can be adjusted to improve performance for different videos.

---

# Required Functions

The following helper functions must be available in your MATLAB path:

- `munkres.m`  
  Implementation of the Hungarian algorithm

- `systematic_resample.m`  
  Particle filter resampling method

If they are not present, the script will not run.

---

# MATLAB Toolboxes

This project requires:

- Image Processing Toolbox
- Computer Vision Toolbox

---

# Example Applications

This tracker can be used for:

- Object tracking in surveillance videos
- Traffic monitoring
- Biological motion analysis
- Robotics and autonomous systems
- Multi-target tracking research

---

# How to Run

1. Open MATLAB.
2. Run the script.

```
run main_script.m
```

3. Select:
   - Video file
   - Output directory

4. The algorithm will start processing the video and display the tracking results.

Press **q** during execution to stop processing.

---

# Visualization

The visualization shows:

- Red circles → detected objects  
- Cyan dots → particles  
- Colored lines → trajectories  
- White circle → estimated object position  
- Label → object ID

---

# License

This project is provided for **research and educational purposes**.

---



اگر بخواهی، می‌توانم یک نسخه **خیلی حرفه‌ای‌تر برای GitHub (با GIF، تصاویر نتایج، badges و ساختار research‑paper style)** هم بسازم که برای **ریپازیتوری دانشگاهی یا مقاله بسیار قوی‌تر** باشد.
