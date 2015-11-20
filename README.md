# seanmcnealy_samples
Personal Software Samples

edges.py and kalman.py come from a class project tracking hexbugs (https://www.youtube.com/watch?v=KQ9AqM44pI0). Edges tracks a 2D input and after solving for a box shape finds how an agent bounces off those edges based on speed and angle. After training on some data predictions can be made. The kalman filter uses inputs x, y, and internal states x', y', and angular velocity to update and predict the location of a hexbug.
