# ToF_camera_simulator
ToF camara simulator to capture point clouds from STL mesh models

This C++ project implements the simplified Kinect sensor model:

		camera_width = 1280 pixels
		camera_height = 720 pixels
		camera_focal_length = 1160 mm

The C++ codes were implemented based on project "render_kinect" at https://github.com/jbohg/render_kinect

This implementation was simplified with perfect Kinect model without any error or noise.
The implementation focused on simulated scene with multiple cameras. 

If you are using this code, please consider citing:
```
@article{zheng2022automatic,
  title={Automatic identification of mechanical parts for robotic disassembly using the PointNet deep neural network},
  author={Zheng, Senjing and Lan, Feiying and Baronti, Luca and Pham, Duc and Castellani, Marco},
  journal={International Journal of Manufacturing Research},
  volume={17},
  number={1},
  year={2022},
  publisher={Inderscience Publishers (IEL)}
}
```


## Dependencies (tested under Ubuntu20.04)
- Eigen3: sudo apt install libeigen3-dev
- OpenMP: 
- PCL: sudo apt install libpcl-dev
- assimp: sudo apt-get install assimp-utils libassimp5 libassimp-dev
- CGAL: sudo apt-get install libcgal-dev





