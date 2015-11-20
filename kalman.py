#Variables
dt = 1.                                     #time change between each frame

u = matrix([[0.], [0.], [0.], [0.], [0.]])  #external motion. Here, there is none

H =  matrix([[1., 0., 0., 0., 0.],          #measurement function
             [0., 1., 0., 0., 0.],
             [0., 0., 0., 1., 0.]])

R =  matrix([[1., 0., .0],                  #measurement uncertainty
             [0., 1., .0],
             [0., 0., 1.]])

I =  matrix([[]])                           #the identity matrix
I.identity(5)


#calculate the heading direction of the hexbug given two points.
def calculate_heading(prev, next):
    #heading is calculated using arctan( y(t) - y(t-1), x(t) - x(t-1) )
    a = atan2(next[1] - prev[1], next[0] - prev[0])

    #in order to not have too much of a change between headings (can be attributed to sensor error)
    #we ensure there is not more than 2pi of difference between heading values
    while(abs(a - prev[2]) > pi):
        a = a + 2*pi*((prev[2]-a)/abs(prev[2]-a))

    return a


#implenatation of the Kalman Filter
def kalman_filter(x, P, currentMeasurement, lastMeasurement = None):

    #measurement contains the previous values of the hexbug, if they exist. If not, they're set to the currently observed values
    measurement = []
    if lastMeasurement:
        measurement = [lastMeasurement[0], lastMeasurement[1], x.value[3][0]]
    else:
        measurement = [x.value[0][0], x.value[1][0], x.value[3][0]]

    for n in range(len(currentMeasurement)):

        #run the prediction step of the algorithm
        x,P = kalman_prediction(x,P)

        #calculate the new heading based on the current measurements
        heading = calculate_heading(measurement, currentMeasurement[n])

        #set measurement to be the currently observed points and heading
        measurement = [currentMeasurement[n][0], currentMeasurement[n][1], heading]

        #the measurement update step
        Z = matrix([measurement])
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P

    return x,P


#the step of the Kalman filter that predicts the next location of the hexbug given x and P.
def kalman_prediction(x, P, steps=1):
    for n in range(steps):
        a = None
        F = None
        w = x.value[4][0] * 1.

        #w is the angular velocity of the robot. If it is high, we will get more accurate results by using a circular motion model for F
        if abs(w) > 0.5:
            a = x.value[3][0]
            F =  matrix([
                [1., 0., (-sin(a) + sin(a+w*dt)) / w, 0., 0.],
                [0., 1., (cos(a) - cos(a+w*dt)) / w, 0., 0.],
                [0., 0.,               1., 0., 0.],
                [0., 0.,               0., 1., dt],
                [0., 0.,               0., 0., 1.]])

        #if w is not high, we can apply a linear motion model.
        else:
            a = x.value[3][0]
            F =  matrix([
                [1., 0., cos(a), 0., 0.],
                [0., 1., sin(a), 0., 0.],
                [0., 0., 1., 0., 0.],
                [0., 0., 0., 1., dt],
                [0., 0., 0., 0., 1.]])

        #complete the prediction steps of the Kalman Filter algorithm.
        x = (F * x) + u
        P = F * P * F.transpose()

    return x,P

