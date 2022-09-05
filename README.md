<img align="right" height="60" src="https://user-images.githubusercontent.com/5248102/126074528-004a32b9-7911-486a-9e79-8b78e6e66fdc.png">

# photogrammetry_evaluations

This repository contains tools for evaluating photogrammetry results using [COLMAP](https://github.com/colmap/colmap)

![image](https://user-images.githubusercontent.com/5248102/147664343-2aa926fe-9c35-4b02-8dd8-621602c7ea99.png)

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
