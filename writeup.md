

# Implementation details
I implemented the standard PID controller for steering angles as described in the lessons. The actual computation of the steering angle is done in line 45 in PID.cpp, where cte, prev_cte and int_cte are computed in main.cpp.

With the code, I set the PID parameters to the trained parameters from the twiddle algorithm. To see the initial parameter optimization, change line 42 in main.cpp, and set    twiddle_converged to false (line 47 in PID.h).

There is a video showing the car driving in tuned_pid_output.mp4



## Twiddle
To find the hyperparameters for the PID controller, I decided to use twiddle. The twiddle algorithm (line 55-107 in PID.cpp) looks very different from the one described in the lessons, but it is in the end the same. 

Each time we want to test a new iteration of a pid parameter, we have to drive the car a certain amount. I decided to drive for 6000 frames in the start. The amount of frames is increased for each twiddle computation by 2% (line 100 in main.cpp). 

The algorithm struggles to converge to a proper solution and it uses a lot of time .Therefore, I decided to do a very simple search for initial hyperparameters for the PID manually. I chose to use [1.0, 10.0, 0.00004] for the PID variables.

When it converges, it has a average squared CTE error of 0.0247569. The final PID parameters is [0.660301, 15.3279, 0.00120462].

# Reflection

## Describe the effect each of the P, I, D components had in your implementation.

P is the proportional controller, that ensures that the cross track error is low when driving. 

P alone makes the car overshoot often on its track, making the car oscilate. The D is the temporal derivative that ensures that the car counter-steers when the car is already converging to the middle of the lane. That removes the problem of overshooting our lane

PD alone would work if we had no systematic bias; however, a car could have a systematic bias such as a tire being slightly skewed. The I term adjusts to systematic bias over time, by looking at the CTE over time. 

## Describe how the final hyperparameters were chosen.
The details on how we chose the final hyperparameters is given in detail above the reflection chapter. In short summary, we chose a set of rough initial parameters to initialize the twiddle optimization algorithm. Then, for each twiddle iteration we run the car for 5000 frames around the track, and adjust the hyperparameters by looking at the square sum of CTE. 