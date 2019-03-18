from geographiclib.geodesic import Geodesic
from LatLon.lat_lon import LatLon
from LatLon.lat_lon import Latitude
from LatLon.lat_lon import Longitude
import sys
import math
import datetime


#This script takes as input the PCI, and drone coordinates of the past and current measurement
#It outputs

#INPUTS


	#lat_current  #Current Latitude of the drone
	#lon_current  #Current Longitude of the drone
	#lat_past     #Past Latitude of the drone
	#lon_past     #Past Longitude of the drone
	#PCI          #PCI that the drone is connected to
	#SIM          # 0--> Telenor, 1--> TDC
	#speed_past   #Needed in case we loose coordinates (km/h)
	#orientation_past #Orientation past 
	#orientation_current #Orientation new input from the GPS
	#time_pass       #time between calling the function again
	#SIM	      #0 if Telenor or 1 if TDC, 2 for other operator
	#reason       #0 --> "NewCoordinates", #1 --> "NewPCI"
	#folder      #name of the folder to save the file  there
	#acc_x      #accelerometer data in x (to be saved)
	#acc_y      #accelerometer data in y (to be saved)
	#acc_z      #accelerometer data in z (to be saved)
	#orientation_compass #Orientation from compass
	#distance_covered_past  #The distance calculated before (meters)

#OUTPUTS
	#antenna_index
	#lat_current
	#lon_current
	#orientation (to FILE and normal output)
	#speed (to FILE and normal output) hm/h
	#distance (to FILE) in meters
	#reason (to FILE) #This is to have a reference to know after
	
#VALID STATES (coordinates)
	#1 - "No current coordinates, no past coordinates"
	#2 - "No past coordinates"
	#3 - "No current coordinates compute the newer ones"
	#4 - "No current coordinates stay the same because speed is not decent"
	#5 - "Normal situation but moved less_than threshold distance" #NOT OUTPUT
	#6 - "Normal situation, we turned too much but probably because we came from static situation
	#6.5 - "Normal situation, we turned too much.
	#7 - "Normal situation"


#VALID STATES (orientation)
	#8 - "No past orientation, no current orientation"
	#9 - "No current orientaion, use the past one"
	#10 - "No past orientation, rely on the current orientation"

#Defined variables




treshold_dist=0.5 #meters
treshold_angle=110 #degress
treshold_time=3/1000000000 #seconds
speed_thold=3 #km/h

if len(sys.argv)==18: #Because we always have one + the ones we input

	lat_current = sys.argv[1]
	lon_current = sys.argv[2]
	lat_past = sys.argv[3]
	lon_past = sys.argv[4]
	PCI = sys.argv[5]
	SIM = sys.argv[6]
	speed_past = float(sys.argv[7]) #In km/h
	orientation_past = sys.argv[8]
	orientation_current = sys.argv[9]
	time_pass = float(sys.argv[10]) #In nano seconds!
	reason = sys.argv[11]
	folder = sys.argv[12]
	acc_x = float(sys.argv[13])
	acc_y = float(sys.argv[14])
	acc_z = float(sys.argv[15])
	orientation_compass = float(sys.argv[16])
	distance_covered_past=float(sys.argv[17]) #In meters


else:
	print("Number of arguments no valid")
	exit()


if PCI.isdigit(): #Condition just in case the PCI is something strange
	PCI = float(sys.argv[5])
	SIM = float(sys.argv[6])
else:
	print "PCI is not a valid number"
	exit()



#---------------------------------------------------------------------------------------------------
#For the given PCI it searches among all the base stations and gets the BS coordinates


PCI_found="no" #THIS VARIABLE IS TO KNOW IF THE PCI has been found

if SIM==0: #That means that we are using Telenor


	#We get the BS values

	PCI_Telenor = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/PCI_Telenor.txt')]
	Telenor_lat = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/Telenor_lat.txt')]
	Telenor_lon = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/Telenor_lon.txt')]
	for x in PCI_Telenor:
		
		if PCI==x:
			PCI_found="yes"
			index=PCI_Telenor.index(PCI)
			lat_BS=Telenor_lat[index]
			lon_BS=Telenor_lon[index]
			#print("Telenor")


elif SIM==1: #That means that we are using TDC
	
	#We get the BS values
	PCI_TDC = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/PCI_TDC.txt')]
	TDC_lat = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/TDC_lat.txt')]
	TDC_lon = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/TDC_lon.txt')]

	for y in PCI_TDC:
		
		if PCI==y:
			PCI_found="yes"
			index=PCI_TDC.index(PCI)
			lat_BS=TDC_lat[index]
			lon_BS=TDC_lon[index]
			#print("TDC")
else:

#It should never enter here since the Initialize script should handle this situation (SIM=2 for example) and stop automatically all the scripts before starting.

	print("SIM operator value not valid")
	exit()



if PCI_found=="no":
	#for future log purposes
	print "NOT FOUND THE PCI DATA BASE", PCI
	exit()



#Conditions depending if we have the positions or not
def num_there(s):
	return any(i.isdigit() for i in s)



lat_check=num_there(lat_current)
lon_check=num_there(lon_current)
latp_check=num_there(lat_past)
lonp_check=num_there(lon_past)
orientationp_check=num_there(orientation_past)
orientation_check=num_there(orientation_current)







#With this part we already manage to choose the orientation that will be used in the other calculations.

#We handle strange, too different or whatever case for the orientation acquisition

if (not orientationp_check) and (not orientation_check): #We dont have neither past and current orientation
 	#Error, exit
	#This should never happen

	valid=8
	print "NO current and past ORIENTATION available"
	exit()

elif not orientation_check: #We dont have the current orientation ---> use the past one

	valid=9
	drone_orientation=float(orientation_past)

elif not orientationp_check: #We dont have the past orientation --> We can only assume that the current orientation is decent

	valid=10
	drone_orientation=float(orientation_current)


elif orientation_check and orientationp_check: #If we have both correct (we see if the difference it is too much)
	
	orientation_current=float(orientation_current)
	orientation_past=float(orientation_past)

	difference=abs(orientation_current-orientation_past)

	if difference>180:
		difference=360-difference
	if distance_covered_past<treshold_dist and difference>treshold_angle: #There is a big chang but we probably come from a static situation --> rely on new orientation
		drone_orientation=orientation_current
		valid=6
	elif difference>treshold_angle: #Too much change and non static situation before --> we keep the past one
		drone_orientation=orientation_past
		valid=6.5
		
	else: #We think the orientation_current is a realiable value
		valid=7 
		drone_orientation=orientation_current


else: #Any other possible case (?)
	
	print "Error in script, the script should not enter here"
	exit()
	








#Below here we dont need to decide the orientation. It is decided above.





if (not lat_check or not lon_check) and (not latp_check or not lonp_check): #strange case only at the beginning. real problem.

	#This case should never happen provided that we start the measurement when the first two coordinates are available.
	valid=1
	print "NO current and past GPS available"
	exit()
	
elif not latp_check or not lonp_check: #This should not happen since we have estimated the past coordinate.
	#This case should never happen provided that we start the measurement when the first two coordinates are available.
	valid=2
	print "GPS position recovered but we don't have the past one"
	exit()
	
elif (not lat_check or not lon_check) and speed_past>speed_thold: #we only don't have the current position (but yes the past) and the speed was decent. This should be the normal case:

	valid=3
	#print "NO current GPS position"
	lat_past = float(lat_past)
	lon_past = float(lon_past)

	#We should estimate the current position accoording to the last position, the speed, the time has passed and the orientation.

	#Supossing we get the values
	
	speed=speed_past
	#drone_orientation=orientation_past #degrees and it is the past one but will maintain
	distance_covered=(time_pass/1000000000)*(speed_past/3.6) #distance in meters

	j = Geodesic.WGS84.Direct(lat_past,lon_past,drone_orientation,distance_covered)

	lat_current= j['lat2']
	lon_current= j['lon2']

	#print "The estimated new position is lat_current= ", lat_current, " and lon_current= ", lon_current
elif (not lat_check or not lon_check): 
	#we only don't have the current position (but yes the past) and the speed was negligible - keep the same coordinates
	lat_past = float(lat_past)
	lon_past = float(lon_past)
	valid=4
	speed=speed_past
	lat_current=lat_past
	lon_current=lon_past
	#drone_orientation=orientation_past 
	distance_covered=0

elif lat_check and lon_check and latp_check and lonp_check: #Normal situation where you have the past and present positions:

	lat_current = float(lat_current)
	lon_current = float(lon_current)
	lat_past = float(lat_past)
	lon_past = float(lon_past)
	
	# Compute the driven distance 
	#Other way (the good one)
	past = LatLon(Latitude(lat_current), Longitude(lon_current))  
	current = LatLon(Latitude(lat_past), Longitude(lon_past)) 
	distance_covered = past.distance(current, ellipse = 'sphere')*1000 # WGS84 distance in m

	# If covered distance is really small (we suspect that we didn't move)
	if distance_covered<treshold_dist:

		drone_orientation=orientation_past
		speed=speed_past
		valid=5
	else: # If covered distance is NOT small (decent)
		
		speed=(distance_covered/(time_pass/1000000000))*3.6 #In km/h
		
		#valid 6 and 7 already considered
	
		#g = Geodesic.WGS84.Inverse(lat_past,lon_past,lat_current,lon_current)
		#distance=g['s12'] #distance between coordinate points #Old method
		#spericlength=g['a12'] #value in degrees. Could be used for calculating the distance inother way
		#drone_orientation=g['azi1'] #orientation of the drone 					
		#drone_orientation=orientation_past

		#if drone_orientation<0:
		#	drone_orientation=360+drone_orientation


		#difference=abs(drone_orientation-orientation_past)
		#if difference>180:
		#	difference=360-difference
		#if difference>treshold_angle and distance_covered<2*treshold_dist:
		#	drone_orientation=orientation_past 
		#	valid=6
		#else:
		#	valid=7 #This was before in the else.

		#Calculation distance between coordinates manually
		#Re=6371000
		#new_distance=Re*spericlength*math.pi/180	
	
else:
	print "No other option in lat long computation"
	exit()


#After calculating all the necessary distances, coordinates orientation,etc
#Calculating the angle to point 

h = Geodesic.WGS84.Inverse(lat_current,lon_current,lat_BS,lon_BS)
angle_BS=h['azi1']
distance_to_BS=h['s12'] #distance between drone and BS

if drone_orientation>angle_BS:
		angle_to_point=360-drone_orientation+angle_BS
elif drone_orientation<angle_BS:
		angle_to_point=angle_BS-drone_orientation
elif drone_orientation==angle_BS:
		angle_to_point=0

#Just in case
if angle_to_point<0:
	angle_to_point=360+angle_to_point

antenna_index=int(math.floor(angle_to_point/60)); #Posible indexes= 0,1,2,3,4,5

#Index 0 --> 0-60 degrees
#Index 1 --> 60-120 degrees
#Index 2 --> 120-180 degrees
#Index 3 --> 180-240 degrees
#Index 4 --> 240-300 degrees
#Index 5 --> 300-360 degrees

#print "Drone orientation", drone_orientation, " degrees"
#print "Angle to point (wrt to north)", angle_BS, " degrees"
#print "Angle to point (wrt to drone",angle_to_point, " degrees"
#print "Distance ", distance, " m"
#print "Distance manual method", new_distance, " m"
#print "Distance LatLon library", distance_covered, " m (THIS IS THE MORE ACCURATE)"
#print "Speed=", speed, " m/s"
#print "Antenna index is", antenna_index



#Printing the real outputs

print antenna_index
print lat_current
print lon_current
print drone_orientation
print speed
print distance_covered
print valid

#print distance_covered

#print angle_to_point

file="/home/upboard/Desktop/UpBoard-Code-and-Results/Results/"+folder+"/estimations_python.txt"

#We output speed and distance to a file

date= datetime.datetime.now()
speed_distance_file = open(file,"a")
#speedfile.write(repr(speed))
print >>speed_distance_file, date
print >>speed_distance_file, "the estimated distance covered was (in meters)"
print >>speed_distance_file, distance_covered
print >>speed_distance_file, "the estimated speed was (m/s)"
print >>speed_distance_file, speed
print >>speed_distance_file, "the reason of execution was"
print >>speed_distance_file, reason
print >>speed_distance_file, "Normal condition or nan?"
print >>speed_distance_file, valid
print >>speed_distance_file, "Antennaindex selected"
print >>speed_distance_file, antenna_index
print >>speed_distance_file, "Drone orientation (degrees wrt north):"
print >>speed_distance_file, drone_orientation
print >>speed_distance_file, "Estimated distance to BS (in meters)"
print >>speed_distance_file, distance_to_BS
print >>speed_distance_file, "The time passed (in seconds) since last call is"
print >>speed_distance_file, time_pass/1000000000
print >>speed_distance_file, "The angle to point (to BS)"
print >>speed_distance_file, angle_to_point
print >>speed_distance_file, "The latitude (GPS input or estimated)"
print >>speed_distance_file, lat_current
print >>speed_distance_file, "The longitude (GPS input or estimated)"
print >>speed_distance_file, lon_current
print >>speed_distance_file, "Accelerometer x"
print >>speed_distance_file, acc_x
print >>speed_distance_file, "Accelerometer y"
print >>speed_distance_file, acc_y
print >>speed_distance_file, "Accelerometer z"
print >>speed_distance_file, acc_z
print >>speed_distance_file, "Compass orientation"
print >>speed_distance_file, orientation_compass
print >>speed_distance_file, "\r"
speed_distance_file.close()


