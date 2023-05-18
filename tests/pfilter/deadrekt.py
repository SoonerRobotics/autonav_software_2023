import numpy as np

class DeadReckoning:
    def __init__(self) -> None:
        self.xsum = 0
        self.ysum = 0
        self.thetasum = 0
    
    def feedback(self, feedback) -> list[float]:
        self.xsum = self.xsum + float(feedback[1]) * np.cos(self.thetasum) + float(feedback[2]) * np.sin(self.thetasum)
        self.ysum = self.ysum + float(feedback[1]) * np.sin(self.thetasum) + float(feedback[2]) * np.cos(self.thetasum)
        self.thetasum += float(feedback[3])
        return [self.xsum, self.ysum, self.thetasum]