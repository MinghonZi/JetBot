ioctl_i2c_rdwr:
	g++-11 -Wall -Wextra -Wpedantic -std=c++20 -o ioctl_i2c_rdwr ioctl_i2c_rdwr.cc

plain_i2c_rdwr:
	g++-11 -Wall -Wextra -Wpedantic -std=c++20 -o plain_i2c_rdwr plain_i2c_rdwr.cc

libi2c_smbus:
	g++-11 -Wall -Wextra -Wpedantic -std=c++20 -o libi2c_smbus libi2c_smbus.cc -li2c

clean:
	rm -f ioctl_i2c_rdwr
	rm -f plain_i2c_rdwr
	rm -f libi2c_smbus
