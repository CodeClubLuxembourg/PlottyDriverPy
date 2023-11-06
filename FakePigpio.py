class FakePigpio:

    OUTPUT = 'OUTPUT'
    
    def __init__(self):
        self.servo_pulse_width = {}
        
    def pi(self):
        return self

    def set_mode(self, servo, mode):
        pass

    def set_PWM_frequency(self, servo, freq):
        pass

    def set_servo_pulsewidth(self, servo, pulse_width):
        self.servo_pulse_width[servo] = pulse_width
        # Send command to your simulator (maybe add to a queue to be sent later)

        