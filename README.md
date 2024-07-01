<img align="right" height="60" src="https://user-images.githubusercontent.com/5248102/126074528-004a32b9-7911-486a-9e79-8b78e6e66fdc.png">

# photogrammetry_evaluations

This repository contains tools for evaluating photogrammetry results using [COLMAP](https://github.com/colmap/colmap)

![rviz_screenshot_2023_11_21-13_15_28](https://github.com/Jaeyoung-Lim/photogrammetry-evaluations/assets/5248102/5bf7bae7-13fb-41f2-ba35-74bc908fd493)


## Citation
This package was developed for the paper listed below. If you find this package useful, please consider citing the following.
- Lim, Jaeyoung, Nicholas Lawrance, Florian Achermann, Thomas Stastny, Rik Bähnemann, and Roland Siegwart. "Fisher information based active planning for aerial photogrammetry." In 2023 IEEE International Conference on Robotics and Automation (ICRA), pp. 1249-1255. IEEE, 2023.
\[[paper](https://ieeexplore.ieee.org/document/10161136/)\]

```
@inproceedings{lim2023fisher,
  title={Fisher information based active planning for aerial photogrammetry},
  author={Lim, Jaeyoung and Lawrance, Nicholas and Achermann, Florian and Stastny, Thomas and B{\"a}hnemann, Rik and Siegwart, Roland},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={1249--1255},
  year={2023},
  organization={IEEE}
}
```

## Setup
We use COLMAP for the Multiview Stereo pipeline. To install, follow the installation [instructions](https://colmap.github.io/install.html).
```
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout dev
mkdir build
cd build
cmake ..
make -j
sudo make install
```

[^note]: In case of using ENU coordinates for model alignment, colmap `3.7` or newer is required. (https://github.com/colmap/colmap/pull/1371)

## Photogrammetry Reconstructions
Photogrammetric reconstruction for a dataset can be run with the following command
```
make reconstruct [path=<path to dataset>]
```

## Incremental Photogrammetry Reconstructions
Photogrammetric reconstruction with incrementally increasing the number of images can be run with the following command
```
make increment [path=<path to dataset>]
```
The reconstruction results can be found in the `./output` directory at the root of the repository.

To clean the results, run 
```
make clean
```
