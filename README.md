# pyDP
This is an implementation of Douglas-Peucker Algorithm, which can be used for trajectory compression.
When there are enough points in a trajectory, this algorithm can find the key point in the trajectory.
This tool support high dimension points, especially when there are extra messages in your trajectory nodes

## depend on
### numpy 1.11.2, python 3.5.2

## USAGE
### trajectory_compression(raw_trajectory, threshold, distance_index, distance_function)


####raw_trajectory: the trajectory data, exp: [[x1, y1, ...], [x2, y2, ...], ... , [xn, yn, ...]]


####threshold: distance threshold


####distance_index: which fields in a point are considered when calculating distance


####distance_function: method for calculating distance
