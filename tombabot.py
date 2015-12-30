#!/usr/bin/env python3
import json
import random
import socket
import math
from sys import argv

class Point:
  x = None
  y = None
  
  def __init__(self, x, y):
    self.x = x
    self.y = y
    
  def __str__(self):
    return "%6.1f, %6.1f" % (self.x, self.y)
    
  def __eq__(self, obj):
    return obj.x == self.x and obj.y == self.y
  
  def __ne__(self, obj):
    return obj.x != self.x and obj.y != self.y
  
  def __lt__(self, obj):
    return self.x < obj.x and self.y < obj.y
    
  def __le__(self, obj):
    return self.x <= obj.x and self.y <= obj.y
    
  def __gt__(self, obj):
    return self.x > obj.x and self.y > obj.y
    
  def __ge__(self, obj):
    return self.x >= obj.x and self.y >= obj.y
    
  def __add__(self, obj):
    return Point(self.x + obj.x, self.y + obj.y)
    
  def __sub__(self, obj):
    if self.x <= obj.x:
      diffX = obj.x - self.x
    else:
      diffX = self.x - obj.x
    if self.y <= obj.y:
      diffY = obj.y - self.y
    else:
      diffY = self.y - obj.y
    return Point(diffX, diffY)
    
  def __mul__(self, obj):
    if type(obj) is Point:
      return Point(self.x * obj.x, self.y * obj.y)
    elif type(obj) is int:
      return Point(self.x * obj, self.y * obj)
    else:
      raise TypeError
    
  def __round__(self, *args, **kwargs):
    return Point(round(self.x,*args, **kwargs),round(self.y,*args,**kwargs))

class Line:
  a = None
  b = None
  points = None
  
  def __init__(self, pointA, pointB):
    self.a = round(pointA*100) # multiplying by 100 to improve accuracy
    self.b = round(pointB*100)
    
    xcoords = []
    ycoords = []
    
    if self.a.x > self.b.x:
      xrange = self.b.x, self.a.x+1 # this must be stored for the y-calculations below
      for i in range(xrange[0], xrange[1], 100):
        xcoords.append(round(i/100)) # dividing by 100 to get actual pixels
    else:
      xrange = self.a.x, self.b.x+1
      for i in range(xrange[0], xrange[1], 100):
        xcoords.append(round(i/100))
      xcoords.reverse()
      
    if self.a.y > self.b.y:
      stepping = round((len(range(self.b.y,self.a.y))/len(range(xrange[0],xrange[1])))*100)
      if stepping == 0:
        stepping = 1
      for i in range(self.b.y, self.a.y+1, stepping):
        ycoords.append(round(i/100))
    else:
      stepping = round((len(range(self.a.y,self.b.y))/len(range(xrange[0],xrange[1])))*100)
      if stepping == 0:
        stepping = 1
      for i in range(self.a.y, self.b.y+1, stepping):
        ycoords.append(round(i/100))
      ycoords.reverse()
   
    self.points = []
    for x,y in zip(xcoords,ycoords):
      self.points.append(Point(x,y))
    
  def __str__(self):
    strlist = ""
    for point in self.points:
      strlist += str(point)+"\n"
    return strlist
  
class TombaBot:
  mov = {
    'up': 1<<0,
    'down': 1<<1,
    'left': 1<<2,
    'right': 1<<3,
    'space': 1<<4,
    'return': 1<<5
  }
  offsetTable = {
    '-': Point(64,64),
    '|': Point(64,64),
    '/': Point(80,80),
    '\\': Point(80,45),
    ',': Point(45,45),
    '`': Point(45,75),
    '+': Point(64,64)
  }
  wpTable = {
    '-': [Point(20,62),Point(64,62),Point(118,62)],
    '|': [Point(64,20),Point(64,64),Point(64,118)],
    '/': [Point(100,118),Point(118,100)],
    '\\': [Point(118,20),Point(20,118)],
    ',': [Point(20,10),Point(10,20)],
    '`': [Point(10,100),Point(20,118)],
    '+': [Point(64,64)]
  }
  edgeTable = {
    '-': [[Point(1,13),Point(128,13)],[Point(1,110), Point(128,110)]],
    '|': [[Point(112,1),Point(112,128)],[Point(14,1), Point(14,128)]],
    '/': [[Point(114,128),Point(128,17)],[Point(110,128), Point(128,110)]],
    '\\': [[Point(1,108),Point(20,128)],[Point(1,17), Point(112,128)]],
    ',': [[Point(114,1),Point(1,114)],[Point(1,17), Point(17,1)]],
    '`': [[Point(17,1),Point(128,110)],[Point(110,1), Point(128,17)]],
    '+': [[Point(1,17),Point(17,1)],[Point(110,1), Point(128,17)],[Point(128,110), Point(110,128)],[Point(1,110),Point(17,128)]]
  }
  nextTileTable = {
    '-': [(1,0),(-1,0)],
    '|': [(0,1),(0,-1)],
    '/': [(1,0),(0,1)],
    '\\':[(0,-1),(1,0)],
    ',': [(-1,0),(0,-1)],
    '`': [(-1,0),(0,1)],
    '+': [(-1,0),(1,0),(0,-1),(0,1)]
  }
  
  def __init__(self,ip,port):
    self.ip = ip
    self.port = port
    self.currentwaypoint = 0
    self.sticky = False
    self.stickyCount = None
    
  def generate_pixel_tiles(self):
    height = self.initdata['map']['tile_height']
    width = self.initdata['map']['tile_width']
    tiles = self.initdata['map']['tiles']
    rows = []
    for i,tileY in enumerate(tiles):
      row = []
      for j,tileX in enumerate(tileY):
        row.append({'high': Point(i*height, j*width),
                    'low': Point(i*height, j*width)})
      rows.append(row)
    
    self.tilesToPixel = rows
  
  def build_map_edges(self):
    tiles = self.initdata['map']['tiles']
    width = self.initdata['map']['tile_width']
    height = self.initdata['map']['tile_height']
    pixelcount = [0,0]
    self.edgeLines = []
    for row in tiles:
      pixelcount[0] = 0
      for tile in row:
        if tile != '.':
          for line in self.edgeTable[tile]:
            self.edgeLines.append(Line(Point(pixelcount[0] + line[0].x, pixelcount[1] + line[0].y),
                                       Point(pixelcount[0] + line[1].x, pixelcount[1] + line[1].y)))
        pixelcount[0] += width
      pixelcount[1] += height
    
    return self.edgeLines
  
  def rebuild_path(self):
    oldpath = self.initdata['map']['path']
    tiles = self.initdata['map']['tiles']
    newpath = []
    for i,wp in enumerate(oldpath):
      newpath.append(wp)
      if i+1 == len(oldpath):
        j = 0
      else:
        j = i+1
      if len(newpath) > 1:
        k = -2
      else:
        k = -1
      nextTiles = self.nextTileTable[tiles[wp['tile_y']][wp['tile_x']]]
      if (oldpath[j]['tile_x'],oldpath[j]['tile_y']) not in nextTiles:
        newpoints = self.path_finder((newpath[k]['tile_x'],newpath[k]['tile_y']),
                                     (oldpath[i]['tile_x'],oldpath[i]['tile_y']),
                                     (oldpath[j]['tile_x'],oldpath[j]['tile_y']),
                                      tiles)
        #formats the new points:
        for p in newpoints:
          newpath.append({'tile_x':p[0], 'tile_y':p[1]})
      
    self.initdata['map']['path'] = newpath
    print('Path Rebuilt')
  
  def path_finder(self, prev, start, stop, tiles):
    path = []
    prevTile = prev
    currentTile = start
    while stop not in path:
      newTile = self.next_tile_selector(prevTile, currentTile, stop, tiles)
      #print(prevTile, currentTile, newTile)
      path.append(newTile)
      prevTile = currentTile
      currentTile = newTile
    path.pop() #drops the last element since its stop
    #print(path)
    return path
  
  def tiletotile_length(self, tileA, tileB):
    width = self.initdata['map']['tile_width']
    height = self.initdata['map']['tile_height']
    tileApos = Point(tileA[0]*width, tileA[1]*height)
    tileBpos = Point(tileB[0]*width, tileB[1]*height)
    diff = tileApos - tileBpos
    return math.hypot(diff.x, diff.y)
  
  def next_tile_selector(self, prev, start, stop, tiles):
    nexttiles = self.nextTileTable[tiles[start[1]][start[0]]]
    startStopLength = self.tiletotile_length(start,stop)
    shortestTile = None
    shortestLength = None
    for potensialTile in nexttiles:
      newTile = (start[0] + potensialTile[0], start[1] + potensialTile[1])
      newLength = self.tiletotile_length(newTile,stop)
      if newTile[0] == prev[0] \
      and newTile[1] == prev[1]:
        pass
      elif shortestLength != None:
        if newLength < shortestLength:
          shortestLength = newLength
          shortestTile = newTile
      else:
        if newLength < startStopLength:
          shortestLength = newLength
          shortestTile = newTile
        else:
          shortestLength = startStopLength
          shortestTile = stop
    return shortestTile
  
  def sticky_choice(self, choice):
    if self.sticky:
      if self.currentwaypoint == self.stickyCount:
        self.sticky = False
      else:
        choice = self.sticky
    else:
      self.sticky  = choice
      self.stickyCount = self.currentwaypoint + 2
    
    return choice
  
  def waypoint_pos_calc(self, tileOffset=0):
    map = self.initdata['map']
    if self.currentwaypoint + tileOffset > len(map['path']):
      wpCounter = self.currentwaypoint + tileOffset - len(map['path'])
    elif self.currentwaypoint + tileOffset < 0:
      wpCounter = len(map['path']) + tileOffset + self.currentwaypoint
    else:
      wpCounter = self.currentwaypoint
    
    wpTile = (map['path'][wpCounter]['tile_x'],
              map['path'][wpCounter]['tile_y'])
    offset = self.offsetTable[map['tiles'][wpTile[1]][wpTile[0]]]
    waypointPos = Point((wpTile[0] * map['tile_width']) + offset.x,
                  (wpTile[1] * map['tile_height'] + offset.y))
    return waypointPos
   
  def waypoint_handler(self, currentPos):
    map = self.initdata['map']
    waypointPos = self.waypoint_pos_calc()
    waypointPosHigh = Point(waypointPos.x + 56 , waypointPos.y + 56)
    waypointPosLow = Point(waypointPos.x - 56 , waypointPos.y - 56)
    
    if currentPos < waypointPosHigh \
    and currentPos > waypointPosLow:
      self.currentwaypoint += 1
      if self.currentwaypoint >= len(map['path']):
        self.currentwaypoint = 0
      print('Waypoint:', self.currentwaypoint)
      
    return self.waypoint_pos_calc()
  
  def direction_eval(self, currentDirection, waypointDirection, nextWaypointDirection):
    wpDirectionAvg = (waypointDirection + nextWaypointDirection)/2
    differenceDirection = wpDirectionAvg - currentDirection
    
    if differenceDirection < 0:
      pi = -math.pi
    else:
      pi = math.pi
    
    if differenceDirection > pi:
      return self.mov['right']+self.mov['up']
    elif differenceDirection < pi:
      return self.mov['left']+self.mov['up']
    else:
      return self.mov['up']

  def radar_eval(self, previousChoice, currentPos, currentDirection, currentVelocity, nextWaypointDirection):
    futurePos = self.direction_to_pos(currentPos, currentDirection, currentVelocity)
    rangelist = Line(currentPos, futurePos)
    obstacleDetected = False
    outOfBounds = False
    for point in rangelist.points:
      obstacleDetected = self.is_in_modifier('mud', point)
      obstacleDetected = self.is_in_modifier('oil', point) if not obstacleDetected else obstacleDetected
      obstacleDetected = self.is_in_modifier('banana', point) if not obstacleDetected else obstacleDetected
      if obstacleDetected:
        print('Avoiding obstacle!')
        break
      if self.is_outofbounds(point):
        outOfBounds = True
        print('Edge detected.')
    if self.is_in_modifier('mud', currentPos) \
    or self.is_in_modifier('oil', currentPos):
      return self.mov['up']
    if obstacleDetected:
      if not outOfBounds:
        return self.sticky_choice(self.left_right(obstacleDetected, currentPos, currentDirection))
      else:
        #return self.left_right(obstacleDetected, currentPos, currentDirection)
        return previousChoice
    else:
      return previousChoice
  
  def power_eval(self, car):
    if car['powerup'] != 'none':
      print(car['powerup'])
      return self.mov['return']
    else:
      return 0
  
  def left_right(self, modifierPos, currentPos, currentDirection):
    width = self.initdata['map']['tile_width']
    height = self.initdata['map']['tile_height']
    pos = modifierPos[1]
    low = modifierPos[0]
    high = modifierPos[2]
    
    inTile = (math.floor(pos.x / width) , math.floor(pos.y / height))
    correctionTarget = Point((inTile[0]*width) + width/2 , (inTile[1] * height + height/2))
    correctionDirection = self.pos_to_direction(currentPos, correctionTarget)
    
    directionCheck = self.direction_eval(currentDirection, correctionDirection, correctionDirection) - self.mov['up']
    return directionCheck
  
  def is_in_modifier(self, modifier, point):
    mods = self.initdata['map']['modifiers']
    items = self.jsondata['items']
    combined = mods + items
    for mod in combined:
      if mod['type'] == modifier:
        thismodifier = mod
    
        offset = thismodifier['width']/2 ,thismodifier['height']/2
        modPosLow = Point(thismodifier['x'] + offset[0] - (thismodifier['width']/2),
                          thismodifier['y'] + offset[1] - (thismodifier['height']/2))
        modPosHigh = Point(thismodifier['x'] + offset[0] + (thismodifier['width']/2),
                           thismodifier['y'] + offset[1] + (thismodifier['height']/2))
        
        if point <= modPosHigh \
        and point >= modPosLow:
          return (modPosLow,Point(thismodifier['x']+offset[0], thismodifier['y']+offset[1]),modPosHigh)      
    return False
  
  def is_outofbounds(self, point):
    for line in self.edgeLines:
      for p in line.points:
        if p == point:
          return True
    return False
  
  def pos_to_direction(self, currentCords, targetCords):
    trianglesideA = targetCords.x - currentCords.x
    trianglesideB = currentCords.y - targetCords.y
    return math.atan2(trianglesideB , trianglesideA)
  
  def direction_to_pos(self, currentPos, currentDirection, currentVelocity):
    return Point(currentPos.x + math.cos(currentDirection) * currentVelocity,
                 currentPos.y - math.sin(currentDirection) * currentVelocity)
  
  def find_car(self,jsondata, id):
    for car in jsondata['cars']:
      if car['id'] == id:
        return car
    else:
      raise Error('Car not found')
  
  def evaluate_move(self, srvdata):
    self.jsondata = json.loads(srvdata)
    myID = self.initdata['id']
    car = self.find_car(self.jsondata, myID)
    currentPos = Point(car['pos']['x'],car['pos']['y'])
    currentDirection = math.atan2(-car['direction']['y'],
                                   car['direction']['x'])
    currentVelocity = math.hypot(abs(car['velocity']['x']),
                                abs(car['velocity']['y']))
    waypointPos = self.waypoint_handler(currentPos) # Updates waypointing based on current position
    nextWaypointPos = self.waypoint_pos_calc(1) #Offset the waypoint counter by +1
    waypointDirection = self.pos_to_direction(currentPos, waypointPos)
    nextWaypointDirection = self.pos_to_direction(currentPos, nextWaypointPos)
    
    movementChoice = self.direction_eval(currentDirection, waypointDirection, nextWaypointDirection)
    movementChoice = self.radar_eval(movementChoice, currentPos, currentDirection, currentVelocity, nextWaypointDirection)
    powerChoice = self.power_eval(car)
    
    return movementChoice + powerChoice
    
  def run(self):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((self.ip, self.port))

    s.send('TombaBot'.encode('UTF-8'))
    data = s.recv(4096)
    self.initdata = json.loads(data.decode('UTF-8'))
    self.build_map_edges()
    self.rebuild_path()
    self.generate_pixel_tiles()
    s.send(str(self.mov['return']).encode('UTF-8')) # Tell the server we are ready
    while True:
      msg = s.recv(4096)
      s.send(str(self.evaluate_move(msg.decode('UTF-8'))).encode('UTF-8'))

if __name__ == "__main__":
  if argv[1]:
    ip = argv[1]
  else:
    ip = "localhost"
  if argv[2]:
    port = int(argv[2])
  else:
    port = 31337
  bot = TombaBot(ip,port)
  bot.run()
