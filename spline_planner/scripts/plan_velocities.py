# Constants:
#   amax : maximum possible acceleration
#   vmax : maximum possible velocity
#   vmin : slowest we would ever consider going

def curvature(path,i):
 return abs(deriv2(path,i) / max(0.001, pow(deriv1(path,i),2)))

def centripetal(curvature,v):
 return abs(curvature * pow(v,2))

def safe_speed(curvature, v_neighbor, ds):
 cent = centripetal(curvature,v_neighbor)
 if cent >= amax:
   return max(vmin, min(vmax, sqrt(abs(amax / curvature))))
 remaining_acceleration = sqrt(pow(amax,2) - pow(cent,2))
 dt = ds / max(vmin, v_neighbor)
 return max(vmin, min(vmax, v_neighbor + dt * remaining_acceleration))

def path_velocities(path, v_initial, v_final, ds):
 # path is the spline
 # v_initial is speed at beginning of path (current drone speed)
 # v_final is desired speed at end of the path
 # ds is the arc distance between path[i] and path[i+1]
 velocities = ... # initialize as list of numbers with same length (f) as path
 velocities[0] = v_initial
 # Forward iteration ensures each velocity is attainable based on previous speed
 for i in range(f-1):
   velocities[i+1] = safe_speed(curvature(path,i), velocities[i], ds)
 velocities[f] = min(v_final, velocities[f])
 # Backward iteration ensure each velocity allows for upcoming turns
 for i in range(f-1):
   j = f - 2 - i
   velocities[j] = safe_speed(curvature(path,j), velocities[j+1], ds)
 return velocities

