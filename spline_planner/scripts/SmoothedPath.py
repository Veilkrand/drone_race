"""
This class is responsible for doing the dirty-job: fit path from given waypoints and provides
querying methods for the fit path
"""

import math
import operator
from spline_planner.eigen_spline import CubicSpline3DWrapper
import numpy as np

class SmoothedPath(object):

  def fit(self, points, derivatives = None, indices = None):
    """
    Fit a path with the given points
    """
    # similar to points_to_spline in ros_geometry.py, this method fit a cubic BSpline with the given points
    #
    # the difference is this method use splrep instead of interp1d with kind='cubic'
    #
    # splprep/spalde from FITPACK library offers derivatives querying methods which is useful for later processing
    #
    # see: https://docs.scipy.org/doc/scipy/reference/tutorial/interpolate.html#spline-interpolation
    if derivatives is not None and indices is not None and len(derivatives) == len(indices):
      self.spline = CubicSpline3DWrapper(points, derivatives, indices)
    else:
      self.spline = CubicSpline3DWrapper(points)

  def visit_at_interval(self, ds, callback_fn, lookahead_max=-1):
    """
    Query the points on the path at the given interval, specified by parameter ds.

    When a point on the path is visited, `callback_fn` will be called with:
    * the visited point
    * first order derivative at the visited point, w.r.t. to distance along the path (a.k.a. arc length)
    * second order derivative at the visited point, w.r.t. to distance along the path (a.k.a. arc length)
    * the approximated distance from the beginning of the path

    At which point it should visit and how long the distance is are approximated vbia numerical integration method.
    """
    result = self.spline.sampleAndCollect(ds, lookahead_max, 0.0005)
    for t, s, derivs in result:
      arr = np.array(derivs)
      callback_fn(arr[0, :], arr[1, :], arr[2, :], s, t)
