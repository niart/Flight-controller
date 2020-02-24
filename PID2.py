import time
class PID:

    def __init__(self, P, I, D):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.error = 0

        self.sample_time = 0.00 #the interval of updtating error
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()


    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0 #The desired value I want
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time


    def update(self, feedback_value):
        feedback_value = self.output # This is because in Yaw control I directly use Yaw to feedback
        self.error = self.SetPoint - feedback_value #Error = input - feedback
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = self.error - self.last_error
        print('output1', self.output)

        if(delta_time >= self.sample_time): # it means if a new lidar data is read in
            self.PTerm = self.Kp * self.error
            self.ITerm += self.Ki * self.error * delta_time # integration of error wrt time
            self.DTerm = self.Kd * delta_error / delta_time # derivative of error wrt time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = self.error
            self.output = self.PTerm + self.ITerm + self.DTerm
            print('output2', self.output)



