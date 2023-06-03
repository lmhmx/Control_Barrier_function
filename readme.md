# Control Barrier Function

This is the code for `Control Barrier Function Based Quadratic Programsfor Safety Critical Systems`. The code is in `Python`. To set up the necessary environment, you need install `cvxpy`, `numpy` and `matplotlib` in your `Python`.

# Experiment Setup

The parameters of the dynamics are set the same as the paper. In our experiment, we change the acceleration at t=20, 30, 40, 50, 60, 70.

In [0,20], the acceleration is 0. Since the speed of the controlled car is 18m/s, which is less than the target speed(22m/s) and the safety function h(x) is larger than 0, a large u is applied to accelerate the controlled car until its speed reaches 22m/s. Then the car remains the speed. Since the lead car's speed is 10m/s, the safety distance decreases to 0 at about 10s. Then the controlled car has to decrease its speed for safety in [10,20].

In [20,30], lead car accelerates. Therefore, the controlled car can accelerate too. Due to the time delay, the safety distance h(x) has a small value.

In [30,40], lead car remains the speed of 20m/s. Therefore, the controlled car remains in 20m/s too. 

In [40,50], lead car accelerates. So the controlled car accelerate just as the lead car. When the lead car's speed is larger than 22m/s(target speed), the controlled car would not increase its speed. Therefore, there safety distance increase.

In [50,60], lead car's speed  is greater than 22m/s. Therefore, their safety distance increases and controlled car remains 22m/s.

In [60,70], lead car deaccelerates. The safety distance decreases when the lead car's speed is less than 22m/s. 

In [70,100], since the lead car's speed is less than 22 m/s, the safety distance between them decreases and goes to 0. After it reaches 0, the speed of the controlled car begin to decrease its speed until it reaches the same speed as the lead car.
