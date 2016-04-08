#! /usr/bin/env python

class Measure:
  def __init__ (self, debug_mode=False):
    import picamera
    from multiprocessing import Pipe, Process
    self.measurements = []
    self.debug_mode = debug_mode
    self.parent_conn, self.child_conn = Pipe()
    self.p = Process(target=self.do_measure, args=(self.child_conn,))
    self.p.start()
  
  # Send kill command to child process
  def kill (self):
    self.parent_conn.send('STOP')
    #self.parent_conn.recv()
    self.p.join()
  
  # Checks if pipe has any data
  # If it does return that
  # Else return []
  def get_measurement (self):
    if self.parent_conn.poll(0.1) == False:
      return []

    # Else
    measures = self.parent_conn.recv()
    self.measurements += measures
    returnable = self.measurements
    self.measurements = []
    return returnable
  
  def do_measure (self, child_conn):
    import picamera, subprocess, time
    from image_to_angle import get_angle
    camera = picamera.PiCamera()
    
    while (True):
      try:
          measurements = []
          
          # 1. Get camera image 
          if self.debug_mode:
              filename = 'image-{}.jpg'.format(int(time.time()))
          else:
              filename = 'image.jpg'
          #print 'capturing to {}'.format(filename)
          #print 'initted'
          camera.capture(filename)
          #print 'captured'
          
          # 2. Run april tags
          cmd = './measure/april -d {}'.format(filename).split()
          p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
          output = p.communicate()[0]
          for line in output.split('\n'):
            parts = line.strip().split()
            if len(parts) == 0: continue
            tag_id = int(parts[0])
            pixel_loc = float(parts[1])
            
            # 3. Use image to angle
            angle = get_angle(100, pixel_loc)
            measurements.append({
              'id': tag_id,
              'bearing': angle[0]
            })
          
          child_conn.send(measurements)
          if child_conn.poll(0.1):
            s = child_conn.recv()
            if s == 'STOP': break
      except KeyboardInterrupt:
          break
    camera.close()
  
if __name__ == '__main__':
  import time
  
  m = Measure()
  for i in range(30):
    measurements = m.get_measurement()
    print 'T={0}, Measurements={1}'.format(i, measurements)
    time.sleep(0.25)
  m.kill()
