
class LowPassFilter(object):

  def __init__(self, initial, tau):
    self.value = initial
    self.tau = tau

  def update(self, measurement, dt):
    alpha = dt / (self.tau + dt)
    self.value = alpha * measurement + (1 - alpha) * self.value
    return self.value
