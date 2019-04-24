from geographiclib.geodesic import Geodesic
from LatLon.lat_lon import LatLon
from LatLon.lat_lon import Latitude
from LatLon.lat_lon import Longitude
from numpy import *
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
	#time_pass       #time between calling the function again
	#reason       #0 --> "NewCoordinates", #1 --> "NewPCI"
	#folder      #name of the folder to save the file  there

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
#treshold_angle=110 #degress #Not needed
treshold_time=3/1000000000 #seconds
#speed_thold=3 #km/h #Not needed
drone_orientation=0 #We always head north

if len(sys.argv)==11: #Because we always have one + the ones we input
	lat_current = sys.argv[1]
	lon_current = sys.argv[2]
	lat_past = sys.argv[3]
	lon_past = sys.argv[4]
	PCI = sys.argv[5]
	SIM = sys.argv[6]
	speed_past = float(sys.argv[7]) #In km/h
	time_pass = float(sys.argv[8]) #In nano seconds!
	reason = sys.argv[9]
	folder = sys.argv[10]
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
two_PCIs=0

if SIM==0: #That means that we are using Telenor
	#We get the BS values
	PCI_Telenor = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/PCI_Telenor.txt')]
	Telenor_lat = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/Telenor_lat.txt')]
	Telenor_lon = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/Telenor_lon.txt')]
	
	#We change to array
	PCI_Telenor=array(PCI_Telenor)
	Telenor_lat=array(Telenor_lat)
	Telenor_lon=array(Telenor_lon)
	indexes=where(PCI_Telenor==PCI) #Extract the indexes where the PCI is
	lat_BS=Telenor_lat[indexes]
	lon_BS=Telenor_lon[indexes]

	if len(lon_BS)==0: #Not found so length 0
		PCI_found="no"
	elif len(lon_BS)==1: #One found, normal situation
		lon_BS=lon_BS[0]
		lat_BS=lat_BS[0]
		PCI_found="yes"
	elif len(lon_BS)==2: #Two found
		two_PCIs=1 #We put the flag
		PCI_found="yes"
	else:
		print("Impossible number of PCI found")
		exit()
		
elif SIM==1: #That means that we are using TDC
	
	#We get the BS values
	PCI_TDC = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/PCI_TDC.txt')]
	TDC_lat = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/TDC_lat.txt')]
	TDC_lon = [float(line.rstrip('\n')) for line in open('/home/upboard/Desktop/UpBoard-Code-and-Results/Code/BS_orientation/TDC_lon.txt')]

	#We change to array
	PCI_TDC=array(PCI_TDC)
	TDC_lat=array(TDC_lat)
	TDC_lon=array(TDC_lon)
	indexes=where(PCI_TDC==PCI) #Extract the indexes where the PCI is
	lat_BS=TDC_lat[indexes]
	lon_BS=TDC_lon[indexes]

	if len(lon_BS)==0: #Not found so length 0
		PCI_found="no"
	elif len(lon_BS)==1: #One found, normal situation
		lon_BS=lon_BS[0]
		lat_BS=lat_BS[0]
		PCI_found="yes"
	elif len(lon_BS)==2: #Two found
		two_PCIs=1 #We put the flag
		PCI_found="yes"
	else:
		print("Impossible number of PCI found")
		exit()
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

#Below here we dont need to decide the orientation. It is decided above.
if (not lat_check or not lon_check) and (not latp_check or not lonp_check): #strange case only at the beginning. real problem.
	#This case should never happen provided that we start the measurement when the first two coordinates are available.
	print "NO current and past GPS available"
	exit()	
elif not latp_check or not lonp_check: #This should not happen since we have estimated the past coordinate.
	#This case should never happen provided that we start the measurement when the first two coordinates are available.
	print "GPS position recovered but we don't have the past one"
	exit()

elif (not lat_check or not lon_check): 
	#we only don't have the current position (but yes the past) and the speed was negligible - keep the same coordinates
	lat_past = float(lat_past)
	lon_past = float(lon_past)
	valid=4
	speed=speed_past
	lat_current=lat_past
	lon_current=lon_past
	distance_covered=0
	
elif lat_check and lon_check and latp_check and lonp_check: #Normal situation where you have the past and present positions:
	lat_current = float(lat_current)
	lon_current = float(lon_current)
	lat_past = float(lat_past)
	lon_past = float(lon_past)

	# Compute the driven distance 
	#Other way (the good one)
	current = LatLon(Latitude(lat_current), Longitude(lon_current))  
	past = LatLon(Latitude(lat_past), Longitude(lon_past)) 
	distance_covered = past.distance(current, ellipse = 'sphere')*1000 # WGS84 distance in m

	# If covered distance is really small (we suspect that we didn't move)
	if distance_covered<treshold_dist:
		speed=speed_past
		valid=5
	else: # If covered distance is NOT small (decent)
		speed=(distance_covered/(time_pass/1000000000))*3.6 #In km/h
		valid=6
else:
	print "No other option in lat long computation"
	exit()


#After calculating all the necessary distances, coordinates orientation,etc
#Calculating the angle to point 

############################### FIRST WE CHECK IF WE HAVE TWO PCIS FOUNDS AND IN THAT CASE WE CHOOSE THE COORDINATES OF THE CLOSER BS ############
if two_PCIs==1:
	current_position = LatLon(Latitude(lat_current), Longitude(lon_current))  
	BS_position_1 = LatLon(Latitude(lat_BS[0]), Longitude(lon_BS[0])) 
	BS_position_2 = LatLon(Latitude(lat_BS[1]), Longitude(lon_BS[1])) 
	distance_to_BS_1 = current_position.distance(BS_position_1, ellipse = 'sphere')*1000 # WGS84 distance in m
	distance_to_BS_2 = current_position.distance(BS_position_2, ellipse = 'sphere')*1000 # WGS84 distance in m
	
	if distance_to_BS_1<distance_to_BS_2:
		lat_BS=lat_BS[0]
		lon_BS=lon_BS[0]
	else:
		lat_BS=lat_BS[1]
		lon_BS=lon_BS[1]		

########################################################################################################################

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
angle_to_point=angle_to_point+30
if angle_to_point<0:
	angle_to_point=360+angle_to_point
elif angle_to_point>360:
	angle_to_point=angle_to_point-360
	
antenna_index=int(math.floor(angle_to_point/60)); #Posible indexes= 0,1,2,3,4,5

#Index 0 --> 330-30 degrees
#Index 1 --> 30-90 degrees
#Index 2 --> 90-150 degrees
#Index 3 --> 150-210 degrees
#Index 4 --> 210-270 degrees
#Index 5 --> 270-330 degrees

#Printing the real outputs

print antenna_index
print lat_current
print lon_current
print speed
print distance_covered
print valid

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
print >>speed_distance_file, "\r"
speed_distance_file.close()
