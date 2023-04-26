csi_camera:
	c++ csi_camera.cc -I/usr/include/opencv4/ -o csi_camera -lopencv_core -lopencv_videoio -lopencv_highgui

ioctl_i2c_rdwr:
	c++ -Wall -Wextra -Wpedantic -std=c++20 -g -o ioctl_i2c_rdwr ioctl_i2c_rdwr.cc

plain_i2c_rdwr:
	c++ -Wall -Wextra -Wpedantic -std=c++20 -g -o plain_i2c_rdwr plain_i2c_rdwr.cc

libi2c_smbus:
	c++ -Wall -Wextra -Wpedantic -std=c++20 -g -o libi2c_smbus libi2c_smbus.cc -li2c

clean:
	rm -f csi_camera
	rm -f ioctl_i2c_rdwr
	rm -f plain_i2c_rdwr
	rm -f libi2c_smbus
