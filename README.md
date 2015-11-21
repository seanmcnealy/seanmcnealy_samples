# seanmcnealy_samples
Personal Software Samples

edges.py and kalman.py come from a class project tracking hexbugs (https://www.youtube.com/watch?v=KQ9AqM44pI0). Edges tracks a 2D input and after solving for a box shape finds how an agent bounces off those edges based on speed and angle. After training on some data predictions can be made. The kalman filter uses inputs x, y, and internal states x', y', and angular velocity to update and predict the location of a hexbug.

RNDF_AStar.cpp comes from UF's DARPA Urban Challenge vehicle. An A\* algorithm searches through road waypoints to find a path to a mission point. Over a small horizon (50m radius) the road network graph along with the planned waypoints are sent to much more localized planners (ones that get sensor data). This is from 2006/2007, and I remember implementing the original A\*, but others may have changed the code after that.
