# FAST_LIO_LOCALIZATION

A simple implementation of FAST-LIO-based localization.

Reference to the design of the localization framework in [move_base](http://wiki.ros.org/move_base): use FAST-LIO as the high-frequency local localizer to publish the transformation from `camera_init` frame to `body`(LiDAR) frame. This node aligns point cloud observed by LiDAR with map in a low-frequency (depend on motion, like move_base) and corrects the transformation between `map` frame and `camera_init` frame.

## Acknowledgments

1. The code of the [FAST_LIO](https://github.com/hku-mars/FAST_LIO) is not modified, please refer to the original repository.
2. The code for NDT point cloud alignment is referenced from the ndt_matching package in [Autoware-AI](https://github.com/Autoware-AI/autoware.ai).
3. The NDT used in this project is [ndt_omp](https://github.com/koide3/ndt_omp), faster than the original NDT in pcl.

## Related Works

1. [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) (Actually I wanted to use this project directly at first, but I did not run it successfully and I was too lazy to debug it, so I wrote my own.)