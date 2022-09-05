# photogrammetry_evaluations

This repository contains tools for evaluating photogrammetry results using [COLMAP](https://github.com/colmap/colmap)

![image](https://user-images.githubusercontent.com/5248102/147664343-2aa926fe-9c35-4b02-8dd8-621602c7ea99.png)

## Photogrammetry Reconstructions
Photogrammetric reconstruction for a dataset can be run with the following command
```
make run [path=<path to dataset>]
```

## Incremental Photogrammetry Reconstructions
Photogrammetric reconstruction with incrementally increasing the number of images can be run with the following command
```
make run-subset [path=<path to dataset>]
```
The reconstruction results can be found in the `./output` directory at the root of the repository.

To clean the results, run 
```
make clean
```
