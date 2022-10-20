# ISCAS22-Real-Time Fast Channel Clustering For Lidar Point Cloud

Previous work: ICCVW21-TradiCV-Survey-of-LiDAR-Cluster

## Motivation
Sensors can produce point clouds with
precise 3D depth information that is essential for autonomous vehicles and robotic systems. As a perception task, point cloud clustering algorithms can be applied to segment the points into object instances. In this brief, we propose a novel, hardwarefriendly fast channel clustering (FCC) algorithm that achieves state-of-the-art performance when evaluated using KITTI panoptic segmentation benchmark. Furthermore, an efficient, pipeline hardware architecture is proposed to implement the FCC algorithm on an FPGA. Experiments show that the hardware design can process each LiDAR frame with 64 channels, 2048 horizontal resolution at various point sparsity in 1.93 ms, which is more than 471.5 times faster than running on the CPU. The code will be released to the public via GitHub


## Dataset Organization

    ICCVW21-LiDAR-Panoptic-Segmentation-TradiCV-Survey-of-Point-Cloud-Cluster
    ├──  Dataset
    ├        ├── semanticKITTI                 
    ├            ├── semantic-kitti-api-master         
    ├            ├── semantic-kitti.yaml
    ├            ├── data_odometry_velodyne ── dataset ── sequences ── train, val, test         # each folder contains the corresponding sequence folders 00,01...
    ├            ├── data_odometry_labels ── dataset ── sequences ── train, val, test           # each folder contains the corresponding sequence folders 00,01...
    ├            └── data_odometry_calib    
    ├──  method_predictions ── sequences

## How to run

```` 
```
docker pull pytorch/pytorch:1.7.1-cuda11.0-cudnn8-runtime 
```
````
Install dependency packages:
```` 
```
bash install_dependency.sh
```
````
Compile specific clusters 
```` 
```
cd PC_cluster
cd ScanLineRun_cluster/Euclidean_cluster/depth_cluster/SuperVoxel_cluster
bash prepare_packages.sh/prepare_pybind.sh
bash build.sh
```
````
Note, prepare_packages.sh may redundantly install packages as clusters are supposed to be used independently. 

One can download the predicted validation results of Cylinder3D from here:
https://drive.google.com/file/d/1QkV8zmRaOAgAZse5CGtlmijcLJVnh7XP/view?usp=sharing

We get the prediction of validation 08 sequence by using the provided checkpoint of Cylinder3D. Thanks for sharing the code!

After downloading, unzip the 08 file, put it inside ./method_predictions/sequences/

It looks like ./method_predictions/sequences/08/predictions/*.label

Run the cluster algorithm
```` 
```
python semantic_then_instance_post_inferece.py
```
````
It should keep updating the visualization figure output_example.png, and overwrite predicted labels in ./method_predictions/sequences/08/predictions/

One can unzip 08 again if wants to run the cluster algorithm again.

Some parameters can be tuned in args parser.


After generating the predicted panoptic label on validation set, one can simply run:
````
```
bash evaluation_panoptic.sh
```
````
Some changes of local path may need to be done. Just follow the error to change them, should be easy. 

The reported numbers should be exactly the same as the paper since traditional methods have no randomness. 

## Publication ##
Please cite the paper if you use this code:

```
@ARTICLE{9803249,
  author={Zhang, Xiao and Huang, Xinming},
  journal={IEEE Transactions on Circuits and Systems II: Express Briefs}, 
  title={Real-Time Fast Channel Clustering for LiDAR Point Cloud}, 
  year={2022},
  volume={69},
  number={10},
  pages={4103-4107},
  doi={10.1109/TCSII.2022.3185228}}
  
@inproceedings{zhao2021technical,
  title={A Technical Survey and Evaluation of Traditional Point Cloud Clustering Methods for LiDAR Panoptic Segmentation},
  author={Zhao, Yiming and Zhang, Xiao and Huang, Xinming},
  booktitle={Proceedings of the IEEE/CVF International Conference on Computer Vision},
  pages={2464--2473},
  year={2021}
}


```
