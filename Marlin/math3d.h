#ifndef __MATH3DH
#define __MATH3DH

//namespace math {

class Vector3d {
public:
	float x;
	float y;
	float z;

	Vector3d(const float& x, const float& y, const float& z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector3d(const Vector3d& vec) {
		this->x = vec.x;
		this->y = vec.y;
		this->z = vec.z;
	}

	float magnitude() const;
	Vector3d unitVector() const;
	float dot(const Vector3d& v2) const;
	Vector3d operator-() const;

	friend Vector3d operator*(const float& f, const Vector3d& v) {
		return Vector3d(f * v.x, f* v.y, f*v.z);
	}
	friend Vector3d operator*(const Vector3d& v1, const Vector3d& v2) {
		return Vector3d(v1.y * v2.z - v1.z * v2.y,v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
	}
	friend Vector3d operator+(const Vector3d& v1, const Vector3d& v2) {
		return Vector3d(v1.x + v2.x,v1.y + v2.y, v1.z + v2.z);
	}
	friend Vector3d operator-(const Vector3d& v1, const Vector3d& v2) {
		return Vector3d(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
	}
};
#endif
//}
