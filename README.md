# CarND-Controls-MPC

In this project I'll implement Model Predictive Control to drive the car around the track. The simulator will provide coordinates of the desired car trajectory  and car's current coordinates and orientation. The model will calculate the steering angle and throttle. Additionally, there's a 100 millisecond latency between actuations commands.

<p align='center'>
<img src="https://github.com/vladyslavgozhenko/CarND-MPC-Project/blob/master/pics/mpc_anime.gif" width="480" alt="simulator" />
</p>
The yellow is a polynomial fitted to waypoints and the green line represents the x and y coordinates of the MPC trajectory.

## The Model

MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. Larger values of time-step (discretization) result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".

<p align='center'>
<img src="https://github.com/vladyslavgozhenko/CarND-MPC-Project/blob/master/pics/length-of-trajectory-and-timestep-duration-02.png" width="480" alt="simulator" />
</p>

The blue line is the reference trajectory and the red line the trajectory computed by Model Predictive Control. In this example the horizon has 7 steps, N, and the space in between white pebbles signifies the time elapsed, dt (the picture is from Udacity Self-Driving Car Nanodegree lections).

Model predictive control uses optimizer, that defines input controls and minimizes the cost function. The optimizer analyzes the current state and the constrains, calculates control inputs that brings the vehicle to a new state and the process will be repeated. The solver uses IPOPT library.

The algorithm of the MPC is show here (we will go later to details, how to select the constrains, size of the time steps and the time horizon T):

<p align='center'>
<img src="https://github.com/vladyslavgozhenko/CarND-MPC-Project/blob/master/pics/MPC_model.png" width="480" alt="simulator" />
</p>
MPC algoritm (the picture is from Udacity Self-Driving Car Nanodegree lections).

## Timestep Length and Elapsed Duration (N & dt)

As was mentioned in Udacity Self-Driving Car Engineer Nanodegree, the following logic could applied for selecting T, dt and N:

    T = N * dt

the product of two other variables N and dt defines the prediction horizon T. Ideally T should be as large as possible, while dt should be as small as possible.

The goal of Model Predictive Control is to optimize the control inputs: [δ,a][\delta, a][δ,a]. An optimizer will tune these inputs until a low cost vector of control inputs is found. The length of this vector is determined by N:

    [δ1,a1,δ2,a2,...,δN−1,aN−1] [\delta_1, a_1, \delta_2, a_2 , ..., \delta_{N-1}, a_{N-1}] [δ1​,a1​,δ2​,a2​,...,δN−1​,aN−1​]

Thus N determines the number of variables optimized by the MPC. This is also the major driver of computational cost.

Timestep Duration.

A good approach to setting N, dt, and T is to first determine a reasonable range for T and then tune dt and N appropriately, keeping the effect of each in mind.

Following that logic, I took N=8 and dt=0.2 seconds. With T=8*0.2sec=1.6sec it should be possible to react on the ever changing situation on the road. More frequent (smaller dt) and numerous (higher N) will result in higher computational cost, slower system and higher (more frequent) mechanical pressure on systems components. With higher dt and higher speeds, it won't be possible to adjust/calculate right car controls on time. Additionally, the simulator provides geo-data with approx. 0.2 milliseconds discretization steps, therefore there is no much sense to use smaller dt than 0.2 msec.

## The state, actuators and update equations.

The idea behind the model it to optimize cost function, that measures the offset from the center of the lane (our desired state or the reference). The errors (deviations) from the reference  will be initialized with zeros:

    ref_cte = 0
    ref_epsi = 0

The problem with this approach, that the car will stop, when it reaches the desired state. To avoid that, the car will be penalized for not maintaining reference velocity ref_vel = 35 km/h.

To avoid sharp changes of controls inputs of the the car, the following constrains after some experimentation were found:

    const double c_cte = 0.5; // for CTE correction
    const double c_epsi = 12.0; // for EPSI correction
    const double c_vel = 2.0; // for velocity correction
    const double c_throttle = 20.0; // for throttle
    const double c_steering = 50.0; // for steering
    const double c_t_seq = 2.0; // to minimize changes between throttle commands in (t) and (t+1)
    const double c_s_seq = 400.0; // to minimize changes between steering commands in (t) and (t+1)

In the calculations speed were used in m/s and steering angles in radians, so the appropriate conversions were done.

## Latency

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay (latency) might be on the order of 100 milliseconds.

A contributing factor to latency is actuator dynamics. For example the time elapsed between when you command a steering angle to when that angle is actually achieved. This could easily be modeled by a simple dynamic system and incorporated into the vehicle model. One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.

    // Predict future state with kinematic model and latency (100 milliseconds)
    double px_t1 = v * latency;
    double py_t1 = 0.0;
    double psi_t1 = (v / 2.67) * (-1 * delta) * latency;
    double v_t1 = v + alpha * latency;// coefficient alpha is the throttle
    double cte_t1 = cte + v * sin(epsi) * latency;
    double epsi_t1 = epsi + (v / 2.67) * (-1 * delta) * latency;

    // define the state vector (px, py, and psi are 0)
    // state considered with 100ms of latency.
    Eigen::VectorXd state(6);
    state << px_t1, py_t1, psi_t1, v_t1, cte_t1, epsi_t1;

The system performs quite well with latency 100 milliseconds. To be able to use higher latencies, reference speed should be adjusted (decreased), otherwise the vehicle movements are no so smooth any more as before and the car leaves the track occasionally.

## Websocket Data Input

Hier is described the JSON object send back from the simulator command server.

Fields:

* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
since y is the up-down direction.
* `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation](https://en.wikipedia.org/wiki/Polar_coordinate_system#Position_and_navigation).
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
