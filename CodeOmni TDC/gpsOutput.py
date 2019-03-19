import threading
from gps import *

class GpsReader(threading.Thread):
	def __init__(self):
		global gps_info
		threading.Thread.__init__(self)
		self.running = True
		self.pausescanner = True
		gps_info = gps(mode=WATCH_ENABLE)

	def run(self):
		global gps_info
		global latitude
		global longitude
		global speed
		global gps_time
		while gps.running:
			if gps_info.waiting(timeout=5):
				gps_info.next()
				latitude = gps_info.fix.latitude
				longitude = gps_info.fix.longitude
				speed = gps_info.fix.speed*3.6
				altitude = gps_info.fix.altitude
				track = gps_info.fix.track
				#head = gps_info.fix.head
				gps_time = gps_info.utc
				if gps_time != "":
					#print gps_time
					print latitude
					print longitude
					print speed
					print altitude
					print track #I think this is the track
					#print heading
					return

gps = GpsReader() # create the thread
gps.start() # start it up

