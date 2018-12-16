#include "math3d.h"
#include "Marlin.h"

//using math::Vector3d;
//float saqrt(float x) {
//    float epsilon = 0.0001;
//    if ( x == 1.0) return 1.0;
//    if ( x == 0.0) return 0.0;
//    
//    float lower = 1.0;
//    float upper = x;
//    if (x < 1.0) {
//        lower = 0.0;
//        upper = 1.0;
//    }
//
//    float guess = (lower + upper) / 2.0;
//    float error = guess * guess - x;
//    while ((error > epsilon) || (error < -epsilon)) {
//SERIAL_PROTOCOLPGM("x = ");
//SERIAL_PROTOCOLLN(x);
//SERIAL_PROTOCOLPGM("guess = ");
//SERIAL_PROTOCOLLN(guess * 1000);
//SERIAL_PROTOCOLPGM("lower = ");
//SERIAL_PROTOCOLLN(lower * 1000);
//SERIAL_PROTOCOLPGM("upper = ");
//SERIAL_PROTOCOLLN(upper * 1000);
//        if (error > epsilon) {
//SERIAL_PROTOCOLLN("upper = guess");
//            upper = guess;
//        }
//        else {
//SERIAL_PROTOCOLLN("lower = guess");
//            lower = guess;
//        }
//        guess = (lower + upper) / 2.0;
//        error = guess * guess - x;
//    }
//    return guess;
//}

float Vector3d::magnitude() const {
	return sqrt(this->x*this->x + this->y*this->y + this->z*this->z);
}

Vector3d Vector3d::unitVector() const {
	float magnitude = this->magnitude();
	return Vector3d(this->x/magnitude,this->y/magnitude,this->z/magnitude);
}

float Vector3d::dot(const Vector3d& v2) const {
	return x * v2.x + y * v2.y + z*v2.z;
}

Vector3d Vector3d::operator-() const {
	return Vector3d(-x,-y,-z);
}


