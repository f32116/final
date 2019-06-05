
import csv
from decimal import *

counter = 0

def main():

	global counter

	# Ground Truth and Measurement Path
	file_gt = '/home/ee904-i5-old-pc-1/Desktop/sdc_ws/src/localization_ground_truth/localization_ground_truth_1.csv'
	file_meas = '/home/ee904-i5-old-pc-1/Desktop/sdc_ws/src/localization_ground_truth/answer_1.csv'

	gt_list = []
	gt_list = read_csv(file_gt)

	meas_list = []
	meas_list = read_csv(file_meas)

	print ("\n***Calculation Starts***\n")
	error = calError(gt_list, meas_list)
	print "\nNumber of your frames: %d" %(len(meas_list))

	# Lost frames are defined that 
	# your measurement timestamp are before or after the ground truth
	print "Number of lost frames: %d" %(len(meas_list) - counter)

	print "Number of frames take into account: %d" %(counter)
	print "Total Error: %s" %(error)
	print "Mean Error: %s\n" %(error/counter)
	print ("***Calculation Finished***")

def calError(_gt, _meas):

	global counter

	_error = 0
	_total_error = 0

	for m in range(len(_meas)):
		Notfound = True
		for i in range(len(_gt)):
			meas = _meas[m]
			if( i != len(_gt)-1 and Notfound):
				x1 = _gt[i]
				x2 = _gt[i+1]
				if( x1[0] <= meas[0] and meas[0] < x2[0]):
					_error = interpolation(meas, x1, x2)
					print "meas_stamp: %s error: %s" %(meas[0], _error)
					_total_error+= _error
					counter+=1
					Notfound = False
			elif(i == len(_gt)-1 and Notfound):
				print "meas_stamp: %s error: lost" %(meas[0])
			else:
				continue			

	return _total_error

def interpolation(m , x1 , x2):

	delta = (m[0] - x1[0])/(x2[0] - x1[0])

	gt_x = x1[1] + (x2[1] - x1[1])*delta
	gt_y = x1[2] + (x2[2] - x1[2])*delta
	gt_z = x1[3] + (x2[3] - x1[3])*delta

	_error = (gt_x-m[1])**2 + (gt_y-m[2])**2 + (gt_z-m[3])**2

	return _error

def read_csv(_csv_file):

	_list = []

	f= open(_csv_file, 'r')
	data = csv.reader(f)

	for row in data:
		if row:
			_list.append(row)

	f.close()

	_list = check_data_type(_list)

	return _list


def check_data_type(_list):

	for m in range(len(_list)):
		_list[m][0]=Decimal(_list[m][0])
		_list[m][1]=Decimal(_list[m][1]) 
		_list[m][2]=Decimal(_list[m][2]) 
		_list[m][3]=Decimal(_list[m][3])

	return _list

if __name__ == '__main__':
	main()