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
from CircularMotionKalmanFilter import CircularMotionKalmanFilter

UNCERTAINTY_COVARIANCE = 1000.0

def initializeKalmanFilter():
    kf = CircularMotionKalmanFilter()
    # P would be diagonal with the large constant uncertainty covariance
    kf.P = matrix([
        [UNCERTAINTY_COVARIANCE, 0, 0, 0, 0],
        [0, UNCERTAINTY_COVARIANCE, 0, 0, 0],
        [0, 0, UNCERTAINTY_COVARIANCE, 0, 0],
        [0, 0, 0, UNCERTAINTY_COVARIANCE, 0],
        [0, 0, 0, 0, UNCERTAINTY_COVARIANCE]
    ])
    return kf

def estimate_next_pos(measurement, OTHER=None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements.

    I use an Extended Kalman Filter (EKF) to estimate five parameters:
        1. the x position
        2. the y position
        3. the current heading direction of the robot
        4. the angle the robot turns between measurements
        5. the distance the robot travels between measurements
    """

    # create an instance of the EKF
    if not OTHER:
        OTHER = initializeKalmanFilter()

    # set up the measurement, updates the EKF and predict the next step
    z = matrix([
        [measurement[0]],
        [measurement[1]]
    ])
    OTHER.updateMeasurement(z)
    x, y = OTHER.predict()

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
    """

    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            # print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            # print "Sorry, it took you too many steps to localize the target."
            return localized, ctr
    return localized, ctr

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER=None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER
    return xy_estimate, OTHER

# run multiple trials of the program to get a good sense of its capability
NUM_TRIALS = 100
localized = 0
counter = []
for i in range(NUM_TRIALS):
    # create a test target
    test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
    measurement_noise = 0.2 * test_target.distance
    test_target.set_noise(0.0, 0.0, measurement_noise)
    # run the estimation function to decided if the target is found
    found, ctr = demo_grading(estimate_next_pos, test_target)
    # record the successes!
    if found:
        localized += 1
        counter.append(ctr)
# print out the final results with success rate and the average iteration number
successPercent = 100.0 * localized / NUM_TRIALS
avgCounter = sum(counter) / len(counter)
print 'Localized {:.2f}% of the trials'.format(successPercent)
print 'Averaged {:.1f} iterations.'.format(avgCounter)
