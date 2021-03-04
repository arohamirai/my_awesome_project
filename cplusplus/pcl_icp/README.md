# An overview of pairwise registration

[overview](./assets/block_diagram_single_iteration.jpg)

# Basic ICP
 need do nothing

 # Interactive ICP
 ```
 ./pcl_icp ../ply/monkey.ply
 ```
# incrementally ICP
```
./pcl_icp ../pcd_pairwise/capture000[1-5].pcd
```
# Robust pose estimation of rigid objects
使用FPFH特征描述子进行匹配
```
./pcl_icp ../pcd/chef.pcd ../pcd/rs1.pcd
```
结果分析和使用感受
1、当现场点云中，除了目标还含有其它很多点时，效果不好；
2、根据不同的点云，调整参数，主要是与点云密度相关的参数；
3、下采样不是必要的，可以根据选择来；
4、迭代次数多也不一定效果就好；
5、如果点云匹配效果很差，通过调整参数获得比较好的效果恐怕也很难；
6、可能更适合预处理之后的冗余不多的点云，例如先筛选出一个点云区域，或者经过点云矫正的区域，在这么一个区域内进行匹配，到底还是更适合姿态匹配；
