def get_landmark (idx):
  import numpy as np
  """
  m 0,0, along x
  6-4 from L, W along y, starting at bottom right
  13-11 from 0,0 along y
  7-10 from LW along x going towards 0

  0 - 11 3/8 5 height

  1 - 35 5/8, 5.25 height

  2 - 59.25

  3 - 83

  6 - 16.25

  5 - 39, 1/8

  4 - 59.75

  13 - 18.25

  12 - 43.75

  11 - 63

  7 - 12.5

  8 - 35.14

  9 - 59 3/8

  10 - 84 3/4
  """
  
  W = 2.00025 
  H = 2.40665
  
  landmarks = {
    0: (W, H-0.288925),
    1: (W, H-0.904875),
    2: (W, H-1.50495),
    3: (W, H-2.1082),
    
    4: (1.51765, 0),  
    5: (0.993775, 0),
    6: (0.41275, 0),
    
    7: (0, 0.3175),
    8: (0, 0.89535),
    9: (0, 1.50812),
    10: (0, 2.15265),
    
    11: (W-1.6002, H),
    12: (W-1.11125, H),
    13: (W-0.46355, H)  
  }
  
  return np.array(landmarks[idx])
