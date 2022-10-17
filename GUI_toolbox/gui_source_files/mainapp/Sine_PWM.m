function [pwm1,pwm2,pwm3] = Sine_PWM(time)

       pwm1 = 0.001;
       pwm2 = 0.001;
       pwm3 = 0.3+0.3*sin(pi*(time-3/4*pi));

end