cd ~
python '/home/jin/Data_Capture/IMU_distribute.py'


mv /home/jin/Data_Capture/imu_vn100.log /home/jin/Data_Capture/myImage/OBS
mv /home/jin/Data_Capture/imu_vn100_even.log /home/jin/Data_Capture/myImage/OBS
mv /home/jin/Data_Capture/myImage/OBS /home/jin/Data_Capture/myImage/OBS$(date +%s)

cd /home/jin/Lingqiu_Jin/D400_IMU_02_26_2018/
./imu_vn100_recorder &
./save_data