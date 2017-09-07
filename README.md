# Visual-Odometry

This a simple and normal Viusal Odometry based on Stereo cameara data.

Some modules are based on ORB-SLAM2(https://github.com/raulmur/ORB_SLAM2) and Slambook(https://github.com/gaoxiang12/slambook).

# Tips

1. The input arguments can be found in file operation.txt;
2. There are some visulization modules for test purpose (named as test_xxxxxx). if you want to process them just uncommon relevant code.

# Imporvements in future

There are several things can be implemented into this project to improve performance:
1. Depth filter can be designed to deal with far points, thus improve localization performance in large scale environment;
2. Triangulation process should be added after the pose computation to make system more robust;
3. Track local map(ORB-SLAM2) module should also be considered;
4. Motion model(ORB-SLAM2) should be considered too;
5. Inlcude local optimization and global optimization;
6. Change parameterization for least square problem;
