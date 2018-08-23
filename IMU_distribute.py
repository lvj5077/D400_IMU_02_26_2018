output_file = open("/home/jin/Data_Capture/imu_vn100_even.log", 'wb')
first = 1
count = -1
with open("/home/jin/Data_Capture/imu_vn100.log", 'rb') as imu_file:
	for line in imu_file:
		tokens = line.split('	')
		cur_time = float(tokens[0])
		count=count+1


with open("/home/jin/Data_Capture/imu_vn100.log", 'rb') as imu_file:
	for line in imu_file:
		tokens = line.split('	')
		if (first>0):
			first = 0
			output_file.write(tokens[0]+" "+ tokens[1]+" "+tokens[2]+" "+ tokens[3]+" "+tokens[4]+" "+ tokens[5]+" "+tokens[6]+" "+ tokens[7]+" "+tokens[8]+" "+tokens[9]+"\n")
			timestamp =float(tokens[0])
			dis = 0.005 # (cur_time-timestamp)/count
		else:
			timestamp = timestamp+dis
			output_file.write(str('%12.9f' %timestamp)+" "+ tokens[1]+" "+tokens[2]+" "+ tokens[3]+" "+tokens[4]+" "+ tokens[5]+" "+tokens[6]+" "+ tokens[7]+" "+tokens[8]+" "+tokens[9]+"\n")

output_file.close()


# 1522873955.901295	9.773731	0.407961	0.733773	0.000180	0.001029	0.000104	-130.049271	85.081947	-151.503983	
