### Describe the effect each of the P, I, D components had in your implementation.
#### P(Proportion)  
P is the main component for correcting CTE error. P have a strong effect to change steering angle toward center of the road.

#### I(Integral)
I is the component for removing accumulated error, and helping a car to move toward center of the road.

#### D(Difference)
D is the component for underweighting the effect of P and helping a car to move forward smoothly.

### Describe how the final hyper parameters were chosen.
By using twiddle, I chose the final hyper parameters.  
The final parameters in this project is below.  

-  `P = 0.071`  
-  `I = 0.0584992`  
-  `D = 0.0680043`  

Initial values are `p = (P, I, D) = (0.5, 0,5, 0,5)`, `dp = (dP, dI, dD) = (0.03, 0.03, 0.03)`.   
I define error as `sum(CTE * CTE) / cycles`. cycles is the value of the times the car measure CTE error and update steer angles.  
This system calculates total error through the 3000 cycles, and then compare with the minimum error.  
If sum of dp is under threshold value `0.0005`, final parameters is decided.  

Here is the result for 0.7 throttle.

[![ScreenShot](http://img.youtube.com/vi/K-QHdTBfSbI/0.jpg)](https://youtu.be/K-QHdTBfSbI)
