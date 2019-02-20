# Gate IR markers estimator

Reconstruct 3D pose of gates and local map based on IR markers sensors messages. 

## IR Markers message id position:

[3]----[4]
 |      |
 |      |
[2]----[1]

[1] (1,1,0)
[2] (-1,1,0)
[3] (-1,-1,0)
[4] (-1,1,0)

Gate width is 0.3 m

## Left Camera info:

height: 768
width: 1024
distortion_model: "plum_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [548.4088134765625, 0.0, 512.0, 0.0, 548.4088134765625, 384.0, 0.0, 0.0, 1.0]
R: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
P: [548.4088134765625, 0.0, 512.0, 0.0, 0.0, 548.4088134765625, 384.0, 0.0, 0.0, 0.0, 1.0, 0.0]

```
---
header: 
  seq: 17818
  stamp: 
    secs: 300
    nsecs: 658357386
  frame_id: ''
markers: 
  - 
    landmarkID: 
      data: "Gate1"
    markerID: 
      data: "2"
    x: 482.294433594
    y: 411.180480957
    z: 0.0
  - 
    landmarkID: 
      data: "Gate1"
    markerID: 
      data: "3"
    x: 482.23147583
    y: 423.835540771
    z: 0.0
  - 
    landmarkID: 
      data: "Gate10"
    markerID: 
      data: "2"
    x: 250.704925537
    y: 326.028839111
    z: 0.0
  - 
    landmarkID: 
      data: "Gate10"
    markerID: 
      data: "4"
    x: 249.819946289
    y: 356.374420166
    z: 0.0
  - 
    landmarkID: 
      data: "Gate13"
    markerID: 
      data: "1"
    x: 515.081481934
    y: 383.159545898
    z: 0.0
  - 
    landmarkID: 
      data: "Gate13"
    markerID: 
      data: "2"
    x: 457.159606934
    y: 383.055389404
    z: 0.0
  - 
    landmarkID: 
      data: "Gate13"
    markerID: 
      data: "3"
    x: 456.685638428
    y: 445.45690918
    z: 0.0
  - 
    landmarkID: 
      data: "Gate13"
    markerID: 
      data: "4"
    x: 514.982055664
    y: 445.574615479
    z: 0.0
  - 
    landmarkID: 
      data: "Gate14"
    markerID: 
      data: "1"
    x: 595.559631348
    y: 404.055297852
    z: 0.0
  - 
    landmarkID: 
      data: "Gate14"
    markerID: 
      data: "2"
    x: 575.159606934
    y: 404.017089844
    z: 0.0
  - 
    landmarkID: 
      data: "Gate14"
    markerID: 
      data: "3"
    x: 575.260986328
    y: 425.944396973
    z: 0.0
  - 
    landmarkID: 
      data: "Gate15"
    markerID: 
      data: "2"
    x: 428.312194824
    y: 399.443389893
    z: 0.0
  - 
    landmarkID: 
      data: "Gate15"
    markerID: 
      data: "3"
    x: 427.992095947
    y: 429.735778809
    z: 0.0
  - 
    landmarkID: 
      data: "Gate2"
    markerID: 
      data: "1"
    x: 832.734863281
    y: 91.8552246094
    z: 0.0
  - 
    landmarkID: 
      data: "Gate2"
    markerID: 
      data: "2"
    x: 181.84765625
    y: 91.3585128784
    z: 0.0
  - 
    landmarkID: 
      data: "Gate4"
    markerID: 
      data: "1"
    x: 861.121948242
    y: 356.705383301
    z: 0.0
  - 
    landmarkID: 
      data: "Gate4"
    markerID: 
      data: "2"
    x: 878.191772461
    y: 356.734466553
    z: 0.0
  - 
    landmarkID: 
      data: "Gate8"
    markerID: 
      data: "1"
    x: 621.583557129
    y: 318.383239746
    z: 0.0
  - 
    landmarkID: 
      data: "Gate8"
    markerID: 
      data: "3"
    x: 621.851013184
    y: 346.511688232
    z: 0.0
  - 
    landmarkID: 
      data: "Gate9"
    markerID: 
      data: "1"
    x: 596.543334961
    y: 414.003265381
    z: 0.0
  - 
    landmarkID: 
      data: "Gate9"
    markerID: 
      data: "3"
    x: 596.64465332
    y: 428.852111816
    z: 0.0
---
```