# 事前準備
- `config`フォルダ直下にカメラパラメータを記述したxmlファイルを配置

例)

```
<?xml version="1.0"?>
<opencv_storage>
<intrinsic type_id="opencv-matrix">
  <rows>3</rows>
  <cols>3</cols>
  <dt>f</dt>
  <data>
    4.81062927e+02 0. 9.64496094e+02 0. 4.79965271e+02 5.49338806e+02 0.
    1. 1.</data></intrinsic>
<rotation type_id="opencv-matrix">
  <rows>1</rows>
  <cols>3</cols>
  <dt>f</dt>
  <data>
    2.18865061e+00 -2.13125563e+00 1.15200274e-01</data></rotation>
<translation type_id="opencv-matrix">
  <rows>1</rows>
  <cols>3</cols>
  <dt>f</dt>
  <data>
    9.58436279e+01 3.70939674e+01 4.92913475e+01</data></translation>
<distortion type_id="opencv-matrix">
  <rows>1</rows>
  <cols>4</cols>
  <dt>f</dt>
  <data>
    -1.12504356e-01 7.97265675e-03 -6.25050670e-05 -3.31788935e-04</data></distortion>
</opencv_storage>
```


# How to use
## View the Camera image
```
roslaunch camera-controller camera.launch
```

## View the AR image 
```
roslaunch camera-controller ar.launch
```

## View the Calibrated image
```
roslaunch camera-controller calibration.launch
```