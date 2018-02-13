# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position.
#
# ----------
# GRADING
#
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import robot  # Check the robot.py tab to see how this works.
from math import *
from matrix import matrix # Check the matrix.py tab to see how this works.

# This is the function you have to write. Note that measurement is a
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
from KalmanFilter import KalmanFilter

UNCERTAINTY_COVARIANCE = 1000.0
MEASUREMENT_NOISE = 10.0

def initializeKalmanFilter(m):
    kf = KalmanFilter(3, m)
    # P would be diagonal with the large constant uncertainty covariance
    kf.P = matrix([
        [UNCERTAINTY_COVARIANCE, 0, 0],
        [0, UNCERTAINTY_COVARIANCE, 0],
        [0, 0, UNCERTAINTY_COVARIANCE]
    ])
    # distance and turning angle should be constant, while the heading
    # direction keeps increasing by the turning angle
    kf.F = matrix([
        [1, 0, 0],
        [0, 1, 1],
        [0, 0, 1]
    ])
    # the measurement matrix would be the distance and the direction
    kf.H = matrix([
        [1, 0, 0],
        [0, 1, 0]
    ])
    # the measurement noise of the distance and direction is unknown,
    # so I simply set it to the identity matrix...
    # TODO: find a better estimate of the measurement noise!
    kf.R = matrix([
        [1, 0],
        [0, 1]
    ])
    return kf

def getDistAngle(measurement, prevMeasurement, prevEstimate):
    dx = measurement[0] - prevMeasurement[0]
    dy = measurement[1] - prevMeasurement[1]
    dist = sqrt(dx ** 2 + dy ** 2)
    angle = atan2(dy, dx)
    if prevEstimate:
        prevAngle = prevEstimate.value[1][0]
        prevTurn = prevEstimate.value[2][0]
        if prevTurn > 0 and angle < prevAngle:
            while angle < prevAngle:
                angle += 2 * pi
        elif prevTurn < 0 and angle > prevAngle:
            while angle > prevAngle:
                angle -= 2 * pi
    return dist, angle


def estimate_next_pos(measurement, OTHER=None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements.

    I use a Kalman Filter to estimate three parameters:
        1. the distance the robot travels between measurements
        2. the current heading direction of the robot
        3. the angle the robot turns between measurements
    This allows the transition matrix to be easily constructed, as shown below.
    """

    # default the prediction to be the same position as measurement
    x, y = measurement

    # If the Kalman Filter has not been initialized, create an instance.
    if not OTHER:
        # initializes OTHER to store the Kalman Filter and the measurements
        OTHER = {}
        OTHER['KalmanFilter'] = initializeKalmanFilter(len(measurement))
        OTHER['prevMeasurement'] = False
        OTHER['prevEstimate'] = False
        OTHER['pathLength'] = 0

    OTHER['pathLength'] += 1

    if not OTHER['prevMeasurement']:
        # cannot make good predictions using only one measurement, so just
        # record the measurement in the path
        pass
    elif OTHER['pathLength'] == 2:
        # initialize the estimates in the Kalman Filter after two measurements
        dist, angle = getDistAngle(measurement, OTHER['prevMeasurement'], OTHER['prevEstimate'])
        OTHER['KalmanFilter'].x = matrix([
            [dist],
            [angle],
            [0.0]
        ])
    else:
        # with more than two measurements, start updating the Kalman filter
        # with the new measurements
        dist, angle = getDistAngle(measurement, OTHER['prevMeasurement'], OTHER['prevEstimate'])
        z = matrix([
            [dist],
            [angle]
        ])
        OTHER['KalmanFilter'].updateMeasurement(z)
        estimates, _ = OTHER['KalmanFilter'].predict()
        estimateDist = estimates.value[0][0]
        estimateAngle = estimates.value[1][0]
        x = measurement[0] + estimateDist * cos(estimateAngle)
        y = measurement[1] + estimateDist * sin(estimateAngle)
        OTHER['prevEstimate'] = estimates

    OTHER['prevMeasurement'] = measurement
    return (x, y), OTHER

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading(estimate_next_pos_fcn, target_bot, OTHER=None):
    """
    This is here to give you a sense for how we will be running and grading
    your code. Note that the OTHER variable allows you to store any
    information that you want.
    """
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier = 25.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.1, 0.1, 0.1)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.1, 0.1, 0.1)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.1, 0.1, 0.1)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    #End of Visualization
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
        #More Visualization
        measured_broken_robot.setheading(target_bot.heading*180/pi)
        measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER=None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

demo_grading(estimate_next_pos, test_target)
