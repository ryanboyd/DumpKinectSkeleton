# Kinect Skeleton Joint Data Dump #

## Description

Dumps color camera stream and joints data from the first kinect skeleton found.

The original project can be found here:
https://github.com/sebtoun/DumpKinectSkeleton

I have added a small bit of code that calculates some additional joint info at the time of output, along with hand states.

------------------------------------------------------------------

## Joint output interpretation

### JOINT STATE

|State|Value|Description|
|:---|:---:|:---|
| NotTracked | 0 |	The joint data is not tracked and no data is known about this joint. |
| Inferred | 1 | The joint data is inferred.Confidence in the position data is very low. |
| Tracked | 2 |	The joint data is being tracked and the data can be trusted. |

### JOINT NUMBERS

|Joint Name|Value|Description|
|:---|:---:|:---|
| SpineBase | 0 | Base of the spine |
| SpineMid | 1 | Middle of the spine |
| Neck | 2 | Neck |
| Head | 3 | Head |
| ShoulderLeft | 4 | Left shoulder |
| ElbowLeft | 5 | Left elbow |
| WristLeft | 6 | Left wrist |
| HandLeft | 7 | Left hand |
| ShoulderRight | 8 | Right shoulder |
| ElbowRight | 9 | Right elbow |
| WristRight | 10 | Right wrist |
| HandRight | 11 | Right hand |
| HipLeft | 12 | Left hip |
| KneeLeft | 13 | Left knee |
| AnkleLeft | 14 | Left ankle |
| FootLeft | 15 | Left foot |
| HipRight | 16 | Right hip |
| KneeRight | 17 | Right knee |
| AnkleRight | 18 | Right ankle |
| FootRight | 19 | Right foot |
| SpineShoulder | 20 | Spine at the shoulder |
| HandTipLeft | 21 | Tip of the left hand |
| ThumbLeft | 22 | Left thumb |
| HandTipRight | 23 | Tip of the right hand |
| ThumbRight | 24 | Right thumb |

### HAND STATE

|State|Value|Description|
|:---|:---:|:---|
| Not applicable | -1 | The joint referred to on this line is not a hand. | 
| Unknown | 0 | The state of the hand is unknown. | 
| NotTracked | 1 | Hand state is not tracked. | 
| Open | 2 | The hand is open. | 
| Closed | 3 | The hand is closed. | 
| Lasso | 4 | The hand is in the lasso state. | 


### HAND STATE CONFIDENCE

|State|Value|Description|
|:---|:---:|:---|
| NotTracked | 0 |	Not tracked. |
| Inferred | 1 | Tracked. |

------------------------------------------------------------------

## Lean output interpretation

"Lean" is tracked by the Kinect along 2 axes.

* **lean.left_right** ranges between -1 and +1
  * -1 corresponds to a ~45° lean to the left
  * +1 corresponds to a ~45° lean to the right
  
* **lean.front_back** ranges between -1 and +1
  * -1 corresponds to a ~45° lean to the back
  * +1 corresponds to a ~45° lean to the front

See also: https://msdn.microsoft.com/en-us/library/dn785526.aspx

------------------------------------------------------------------

##Calculating Pitch, Roll, and Yaw from orientation data

This same information is presented as a comment in the code near the bottom (with source link). However, just so you don't have to go mucking through the code to find this information, here are the basic formulae for calculating pitch, roll, and yaw from the orientation data in the output.

```
Pitch = Math.Atan2(2 * ((orientation.Y * orientation.Z) + (orientation.W * orientation.X)), (orientation.W * orientation.W) - (orientation.X * orientation.X) - (orientation.Y * orientation.Y) + (orientation.Z * orientation.Z)) / Math.PI * 180.0

Yaw = Math.Asin(2 * ((orientation.W * orientation.Y) - (orientation.X * orientation.Z))) / Math.PI * 180.0
      
Roll = Math.Atan2(2 * ((orientation.X * orientation.Y) + (orientation.W * orientation.Z)), (orientation.W * orientation.W) + (orientation.X * orientation.X) - (orientation.Y * orientation.Y) - (orientation.Z * orientation.Z)) / Math.PI * 180.0
```
