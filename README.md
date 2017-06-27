# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Model

One of the stiffest challenges in this project was to tune parameters of the cost function and other parameters.
First, a third degree polynomial is fitted to waypoints recieved from the simulator and the cross track error (cte) is obtained by evaluating the polynomial at current x position. Actual state of the vehicle was "shifted" into the future by 100 ms latency.
In car coordinates initial car's position (x,y,psi) is (0,0,0). Speed remains the same as in global coordinates.
When we evaluate cte and epsi, the value of x is taken as zero. Also a negative sign was added to delta after it was observed that the car turns in the opposite direction of the predicted trajectory. The State vector is then passed to the optimizer.
Returned value of delta and acceleration are used to actuate the position of the car.

The cost function parameters were tuned by trial and error method. They were tuned in order to reach maximum speed possible without touching the curb and breaking before turns.
The MPC cost is defined using the cte, epsi and velocity v. The cost also accounts for actuators (delta, a) values and the change in the actuator values as in the code.

For the MPC implementation the result of the Model Predictive Control quiz was used as a starting point. To improve the controller, weights were added to the individual cost terms. This way each part of the cost function can be tuned individually to achieve a smooth result. Cost function is displayed below -

    // Initialize the cost value
        fg[0] = 0;

    // Define the costs
    for (int i = 0; i < N; i++) {
      fg[0] += 4000 * CppAD::pow(vars[cte_start + i] , 2);
      fg[0] += CppAD::pow(vars[epsi_start + i], 2);
      fg[0] += CppAD::pow(vars[v_start+ i] - ref_v, 2);
    }

    // Higher weights mean minimizing the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += 80000 * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start+i], 2);
    }

    // Minimize the value gap between sequential actuations.
    // Higher weights will influence the solver into keeping sequential values closer togther
    for (int i = 0; i < N - 2; i++) {
      fg[0] += 1000 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i +1] - vars[a_start+i], 2);
    }


# State

The state vector (x,y,psi,v, steering angle, acceleration) ->

x: cars x global position
y: cars y global position
psi: vehicle's angle in radians from the x-direction (radians)
v: vehicle's velocity
delta: steering angle
a : acceleration (throttle)

# Actuators
delta: steering angle
a : acceleration (throttle)

## Parameter tuning

# N and dt (Timestamp length and elapsed time):

N is number of timesteps and dt is the time gap between each state. I first used N=10 and dt=0.01 to observe the performance of the model. I also tried N=15 till 25, but this resulted in the model having a farther prediction horizon and the solution would break at the curves. N=10 and dt=0.01 was finally chosen. The choice of dt is also determined by the speed you choose. For fast speed you might want to quick adjustment to control the vehicle. By trial and error, I determined the value N and dt to be 10 and 0.1 and works well.

# Latency

Latency is handled by optimizing the cost function and averaging out the first two actuator values of the solution. This approach was found to be more stable than setting dt > latency.
