#ifndef VEHICLE_STATE_H_
#define VEHICLE_STATE_H_
#include <mutex>
#include <iostream>


namespace skywell {
class VehicleState
{
	public:
		void updateVehicleState(uint64_t _time_stamp,float _roll,float _pitch,
						float _yaw,double _longitude,double _latitude,
						float _x,float _y,int _wheel_angle,
						int _velocity,float _velocity_x,float _velocity_y );
		void getVehicleState(uint64_t &_time_stamp,float &_roll,float &_pitch,
						float &_yaw,double &_longitude,double &_latitude,
						float &_x,float &_y,int &_wheel_angle,
						int &_velocity,float &_velocity_x,float &_velocity_y);
	private:
		uint64_t time_stamp;
		float roll;
		float pitch;
		float yaw;
		double longitude;
		double latitude;
		float x;
		float y;
		int wheel_angle;
		int velocity;
		float velocity_x;
		float velocity_y;
	private:
		std::mutex mtx;
};

}

#endif
