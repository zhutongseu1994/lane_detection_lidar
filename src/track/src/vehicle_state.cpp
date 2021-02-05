#include "vehicle_state.h"


namespace skywell {

void VehicleState::updateVehicleState(uint64_t _time_stamp,float _roll,float _pitch,
						float _yaw,double _longitude,double _latitude,
						float _x,float _y,int _wheel_angle,
						int _velocity,float _velocity_x,float _velocity_y )
{
	mtx.lock();
	 time_stamp = _time_stamp;
	 roll = _roll;
	 pitch = _pitch;
	 yaw = _yaw;
	 longitude = _longitude;
	 latitude = _latitude;
	 x = _x;
	 y = _y;
	 wheel_angle = _wheel_angle;
	 velocity = _velocity;
	 velocity_x = _velocity_x;
	 velocity_y = _velocity_y;
	 mtx.unlock();
};
void VehicleState::getVehicleState(uint64_t &_time_stamp,float &_roll,float &_pitch,
						float &_yaw,double &_longitude,double &_latitude,
						float &_x,float &_y,int &_wheel_angle,
						int &_velocity,float &_velocity_x,float &_velocity_y)
	{
			mtx.lock();
			_time_stamp = time_stamp;
			_roll = roll;
			_pitch = pitch;
			_yaw = yaw;
			_longitude = longitude;
			_latitude = latitude;
			_x = x;
			_y = y;
			_wheel_angle = wheel_angle;
			_velocity = velocity;
			_velocity_x = velocity_x;
			_velocity_y = velocity_y;
			mtx.unlock();
	};
}