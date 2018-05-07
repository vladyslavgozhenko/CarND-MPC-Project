# CarND-PID-Control-Project
<p align='center'>
<img src="https://github.com/wiwawo/CarND-PID-Control-Project/blob/master/pid_control.png" width="480" alt="simulator" />
</p>

In this project I will utilize PID-Control method to calculate steering angle for a car.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

INPUT: values provided by the simulator to the c++ program

["cte", "speed", "steering_angle"] => the measurement that the simulator observs.
"CTE" stands for cross track error (deviation from the center of a road) and will
be used to calculate steering angle for a car. Current "speed" and "steering angle"
won't be used in the program.

OUTPUT: values provided by the c++ program to the simulator

["steer_value"] <= optimum steering angle for a wheel to get to a center of the road

## Algoritm

In PID-Control method, the steering angle is calculated using following formula:

steering_value = -Kp * p_error - Kd * d_error - Ki * i_error,

where:
    p_error - proportional error, which is equal to the current deviation from the
center of a road i.e. p_error = cte;
    d_error - differential error, which is equal to the difference between cte in the
current moment and cte in the previous moment i.e d_error = cte - prev_cte;
    i_error - integral error is sum of all ctes during the whole time of the algorithm
execution i.e. i_error += cte;
    Kp, Kd, Ki are weights(coefficients), that show how important is every error
for calculated steering angles;
    The bigger Kp, the sharper will be turns of the car right away after receiving
new ctes from sensors, the smaller Kp, the smoother will be turning of the car, therefore I choose Kp = 0.05.
    The bigger Kd means, that the higher adjustment of the steering angle will be done, after
the car missed the middle of the road with the steering value -Kp * p_error in the previous moment.
Kd = 0.9 was chosen to more aggressively steer to the middle of the road.
    The bigger Ki means, that the regularly misses middle of the road and this error should be taken in the account.
Since I was satisfied how the algorithms performs with the chosen Kp and Kd, the value for Ki was not important any more
and Ki=0.0001 was chosen.

To avoid sharp turning angles the limit [-1,1] was set for steering angles.

To see how the algorithm performed, please check the following [video](https://github.com/wiwawo/CarND-PID-Control-Project/blob/master/video/pid_control.mp4)

## Twiddle algoritm

I didn't use this algorithm in my program, but it is a good alternative to manually selecting of the coefficients Kp, Kd, Ki.
The idea behind Twiddle method, that some initial values for Kp, Kd and Ki selected. Than we set some areas (ranges)
around those value, where Kp, Kd and Ki can change. If change in a coefficient decreased error i.e. cte in comparison to
errors with previous changes, value the coefficent will be updated and the area will increased, otherwise area around
the coefficient will be decreased and the coefficients value won't be updated.

The Twiddle algorithm in Python:

def twiddle(tol=0.2):
    p = [0, 0, 0]
    dp = [1, 1, 1]
    best_err = err_calc(p)
    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            err_calc(p)
            if err < best_err:
                best_err = err
                dp[i] = 1.1*dp[i]
            else:
                p[i] -= 2 * dp[i]
                err = err_calc(p)
                if err < best_err:
                    best_err = err
                    dp[i] = 1.1*dp[i]
                else:
                    p[i] += dp[i]
                    dp[i] = 0.9*dp[i]
        it += 1
    return p

Results of the Twiddle algoritm look something like this:
<p align='center'>
<img src="https://github.com/wiwawo/CarND-PID-Control-Project/blob/master/twiddle.png" width="480" alt="simulator" />
</p>

## Possible improvents
  * sharp turning angles can be avoided if current speed valus will be taken in account.
Since sharp turning angles happen when the road turn is sharp and the speed is high, it makes
sense to deacrease the speed (and throttle) of the car as soon as the steering angle increases and
do the opposite when the steering angle is close to zero;
  * combining of PID-Control and the optical recognition of lanes (road middle) can help to avoid
  'wiggling' of a car around the middle of a road.

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./pid `
