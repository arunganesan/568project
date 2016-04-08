# https://www.raspberrypi.org/documentation/hardware/camera.md
def get_angle (row, col): # row/col pixel values of pixel 
  import math
  
  # We assume (0,0) is top left of image
  FocalLength = 3.6 # millimeters
  HorizontalView = 53.50 # degrees 
  VerticalView = 41.41 # degrees
  
  Cols = 640; #2592.0
  Rows = 480; #1944.0
  SensorWidth = 3.76 # mm
  SensorHeight = 2.74
  
  # Horizontal
  normalized_col = (1-(Cols - col)/Cols)
  normalized_d_from_cen = 0.5 - normalized_col
  distance_from_center = normalized_d_from_cen * SensorWidth
  horizontal_angle = math.atan2(distance_from_center, FocalLength)
  
  # Vertical
  distance_from_center = (Rows - row)/Rows * SensorHeight
  vertical_angle = math.atan2(distance_from_center, FocalLength)
  
  # Positive is right and up
  return math.degrees(horizontal_angle), vertical_angle

if __name__ == '__main__':
  import math
  
  X1 = 652.973
  X2 = 1299.56
  X3 = 2010.35
  
  Y = 100; #1037.16
  
  print get_angle(Y, X1)
  print get_angle(Y, X2)
  print get_angle(Y, X3)
