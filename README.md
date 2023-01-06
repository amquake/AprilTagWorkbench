# AprilTagWorkbench

**(VERY WIP)**

[wpilib-2023.1.1]
[photonlib-beta-8-5-gbde023c0]

-----

Current progress:

- Simulating a camera (intrinsics, distortion, pixel noise, framerate, latency)
- Simulating photon target info, best / alternate estimated poses and ambiguity from single-tag solvePnP:

https://user-images.githubusercontent.com/7953350/208899056-d454a011-ca3c-4be1-b847-47e5eb36d378.mp4

- Simulating multiple cameras

https://user-images.githubusercontent.com/7953350/208900503-f73108bb-a184-4c09-babc-6531c2686a1e.mp4

- Pose estimation from solvePnP(`SQ_PNP`) with multiple visible tags(from a single camera):

https://user-images.githubusercontent.com/7953350/208900265-95a15722-ea11-44bd-a344-959b82eccc15.mp4

- Pose estimation from relative 3d translations using SVD (possibly multiple cameras)

https://user-images.githubusercontent.com/7953350/208897447-8bbeec43-0c06-4829-9b88-48ef64bc1e9b.mp4

- Pose estimation from trigonometry(with known tag and camera heights) and the above approach:

https://user-images.githubusercontent.com/7953350/208894497-d57316be-3e7c-4d87-90ec-8996faea18ab.mp4

- Simulating camera video streams

https://user-images.githubusercontent.com/7953350/210769846-e5a073dd-0d96-40e7-96b1-418e4818fab5.mp4

------

## Related
- [AprilTagLayoutId](https://github.com/amquake/AprilTagLayoutId)
  - > Estimates a layout of AprilTags by periodically recording the visible tags and their transformations between eachother.
