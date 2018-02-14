# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot.
# But this time, your speed will be about the same as the runaway bot.
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time.
#
# ----------
# GRADING
#
# Same as part 3. Again, try to catch the target in as few steps as possible.

from math import atan2, pi, sqrt
from matrix import matrix
from robot import robot

from CircularMotionKalmanFilter import CircularMotionKalmanFilter

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # create an instance of the EKF, and also keeps track of how many steps
    # afterwards are we planning to capture the robot
    if not OTHER:
        OTHER = {}
        OTHER['EKF'] = CircularMotionKalmanFilter()
        OTHER['steps'] = 0

    # set up the measurement, updates the EKF and predict the next step
    z = matrix([
        [target_measurement[0]],
        [target_measurement[1]]
    ])
    OTHER['EKF'].updateMeasurement(z)
    target_position = OTHER['EKF'].predict()

    # if we did not previously set up a plan, or number of steps after which we
    # will capture the robot, predict several steps ahead and plan to capture
    # the robot when we can cover enough distance
    if OTHER['steps'] < 1:
        # keep predicting the next step until our movements cover the distance
        nSteps = 1
        while distance_between(hunter_position, target_position) > max_distance * nSteps:
            target_position = OTHER['EKF'].predictAfterN(1)
            nSteps += 1
        OTHER['steps'] = nSteps
    else:
        # otherwise, refine the predicted capture position
        OTHER['steps'] -= 1
        target_position = OTHER['EKF'].predictAfterN(OTHER['steps'])

    # turn the direction towards the predicted capture location, and move as
    # much as the max distance or the distance to the target
    heading_to_target = get_heading(hunter_position, target_position)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference # turn towards the target
    distance = min(max_distance, distance_between(hunter_position, target_position))
    return turning, distance, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER=None, plot=False):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    # hunter must be within 0.02 step size to catch target
    separation_tolerance = 0.02 * target_bot.distance
    caught = False
    ctr = 0
    if not plot:
        # We will use your next_move_fcn until we catch the target or time expires.
        while not caught and ctr < 1000:

            # Check to see if the hunter has caught the target.
            hunter_position = (hunter_bot.x, hunter_bot.y)
            target_position = (target_bot.x, target_bot.y)
            separation = distance_between(hunter_position, target_position)
            if separation < separation_tolerance:
                # print "You got it right! It took you ", ctr, " steps to catch the target."
                caught = True

            # The target broadcasts its noisy measurement
            target_measurement = target_bot.sense()

            # This is where YOUR function will be called.
            turning, distance, OTHER = next_move_fcn(
                hunter_position,
                hunter_bot.heading,
                target_measurement,
                max_distance,
                OTHER
            )

            # Don't try to move faster than allowed!
            if distance > max_distance:
                distance = max_distance

            # We move the hunter according to your instructions
            hunter_bot.move(turning, distance)

            # The target continues its (nearly) circular motion.
            target_bot.move_in_circle()

            ctr += 1
            if ctr >= 1000:
                # print "It took too many steps to catch the target."
                return caught, ctr
    else:
        #For Visualization
        import turtle
        window = turtle.Screen()
        window.bgcolor('white')
        chaser_robot = turtle.Turtle()
        chaser_robot.shape('arrow')
        chaser_robot.color('blue')
        chaser_robot.resizemode('user')
        chaser_robot.shapesize(0.3, 0.3, 0.3)
        broken_robot = turtle.Turtle()
        broken_robot.shape('turtle')
        broken_robot.color('green')
        broken_robot.resizemode('user')
        broken_robot.shapesize(0.3, 0.3, 0.3)
        size_multiplier = 15.0 #change size of animation
        chaser_robot.hideturtle()
        chaser_robot.penup()
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        chaser_robot.showturtle()
        broken_robot.hideturtle()
        broken_robot.penup()
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        broken_robot.showturtle()
        measuredbroken_robot = turtle.Turtle()
        measuredbroken_robot.shape('circle')
        measuredbroken_robot.color('red')
        measuredbroken_robot.penup()
        measuredbroken_robot.resizemode('user')
        measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
        broken_robot.pendown()
        chaser_robot.pendown()
        #End of Visualization
        # We will use your next_move_fcn until we catch the target or time expires.
        while not caught and ctr < 1000:
            # Check to see if the hunter has caught the target.
            hunter_position = (hunter_bot.x, hunter_bot.y)
            target_position = (target_bot.x, target_bot.y)
            separation = distance_between(hunter_position, target_position)
            if separation < separation_tolerance:
                print "You got it right! It took you ", ctr, " steps to catch the target."
                caught = True

            # The target broadcasts its noisy measurement
            target_measurement = target_bot.sense()

            # This is where YOUR function will be called.
            turning, distance, OTHER = next_move_fcn(
                hunter_position,
                hunter_bot.heading,
                target_measurement,
                max_distance,
                OTHER
            )

            # Don't try to move faster than allowed!
            if distance > max_distance:
                distance = max_distance

            # We move the hunter according to your instructions
            hunter_bot.move(turning, distance)

            # The target continues its (nearly) circular motion.
            target_bot.move_in_circle()
            #Visualize it
            measuredbroken_robot.setheading(target_bot.heading*180/pi)
            measuredbroken_robot.goto(
                target_measurement[0]*size_multiplier,
                target_measurement[1]*size_multiplier-100
            )
            measuredbroken_robot.stamp()
            broken_robot.setheading(target_bot.heading*180/pi)
            broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
            chaser_robot.setheading(hunter_bot.heading*180/pi)
            chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
            #End of visualization
            ctr += 1
            if ctr >= 1000:
                print "It took too many steps to catch the target."
        return caught, ctr
    return caught, ctr


def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

# run multiple trials of the program to get a good sense of its capability
NUM_TRIALS = 100
nCaptured = 0
counter = []
for i in range(NUM_TRIALS):
    # create a test target
    target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
    measurement_noise = 2.0*target.distance
    target.set_noise(0.0, 0.0, measurement_noise)
    # run the estimation function to decided if the target is captured
    hunter = robot(-10.0, -10.0, 0.0)
    found, it = demo_grading(hunter, target, next_move)
    # record the successes!
    if found:
        nCaptured += 1
        counter.append(it)

if nCaptured > 0:
    # print out the final results with success rate and the average iteration number
    successPercent = 100.0 * nCaptured / NUM_TRIALS
    avgCounter = float(sum(counter)) / len(counter)
    print 'Captured {:.2f}% of the {:d} trials'.format(successPercent, NUM_TRIALS)
    print 'Averaged {:.1f} iterations'.format(avgCounter)
else:
    print 'All {:d} attempt(s) failed!!'.format(NUM_TRIALS)
