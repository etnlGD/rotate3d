// COMPILE: g++ -o quat2EulerTest quat2EulerTest.cpp 
#include <iostream>
#include <cmath> 
#include <cstdio>
#include <cstdlib>

using namespace std;

///////////////////////////////
// Quaternion struct
// Simple incomplete quaternion struct for demo purpose
///////////////////////////////
enum RotSeq{xyz, zyz, yxz, zxz, zxy, yxy, xzy, yzy, zyx, xyx, yzx, xzx, SeqCount};

struct Vec3
{
	double x;
	double y;
	double z;
};

struct Quaternion{
	Quaternion():x(0), y(0), z(0), w(1){};
	Quaternion(double x, double y, double z, double w):x(x), y(y), z(z), w(w){};
	
	void normalize(){
		double norm = std::sqrt(x*x + y*y + z*z + w*w);
		x /= norm;
		y /= norm;
		z /= norm;
		w /= norm;
	}
 
	double norm(){
		return std::sqrt(x*x + y*y + z*z + w*w);
	}
	
	double x;
	double y;
	double z;
	double w; 
 
};

///////////////////////////////
// Quaternion to Euler
///////////////////////////////

void twoaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
	res[0] = atan2( r11, r12 );
	res[1] = acos ( r21 );
	res[2] = atan2( r31, r32 );
}
	 
void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
	res[0] = atan2( r11, r12 );
	res[1] = asin ( r21 );
	res[2] = atan2( r31, r32 );
}


/// res[0] is the angle around first axis.
/// res[1] is the angle around second axis.
/// res[2] is the angle around third aixs.
/// e.g. if rotSeq is xyz, it means that first rotate res[0] radians around 
/// x-axis, then rotate res[1] radians around y-axis, finally rotate res[2]
/// radians around z-aixs.
void quaternion2Euler(const Quaternion& q, double res[], RotSeq rotSeq)
{
	switch(rotSeq){
	case xyz:
		threeaxisrot(2*(q.y*q.z + q.w*q.x),
					 q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
					 -2*(q.x*q.z - q.w*q.y),
					 2*(q.x*q.y + q.w*q.z),
					 q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
					 res);
		break;
	 
	case zyz:
		twoaxisrot(2*(q.y*q.z + q.w*q.x),
				   -2*(q.x*q.z - q.w*q.y),
				   q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
				   2*(q.y*q.z - q.w*q.x),
				   2*(q.x*q.z + q.w*q.y),
				   res);
		break;
				 
	case yxz:
		threeaxisrot(-2*(q.x*q.z - q.w*q.y),
					 q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
					 2*(q.y*q.z + q.w*q.x),
					 -2*(q.x*q.y - q.w*q.z),
					 q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
					 res);
		break;

	case zxz:
		twoaxisrot(2*(q.x*q.z - q.w*q.y),
				   2*(q.y*q.z + q.w*q.x),
				   q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
				   2*(q.x*q.z + q.w*q.y),
				   -2*(q.y*q.z - q.w*q.x),
				   res);
		break;

	case zxy:
		threeaxisrot(2*(q.x*q.y + q.w*q.z),
					 q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
					 -2*(q.y*q.z - q.w*q.x),
					 2*(q.x*q.z + q.w*q.y),
					 q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
					 res);
		break;

	case yxy:
		twoaxisrot(2*(q.x*q.y + q.w*q.z),
				   -2*(q.y*q.z - q.w*q.x),
				   q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
				   2*(q.x*q.y - q.w*q.z),
				   2*(q.y*q.z + q.w*q.x),
				   res);
		break;
	 
	case xzy:
		threeaxisrot(-2*(q.y*q.z - q.w*q.x),
					 q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
					 2*(q.x*q.y + q.w*q.z),
					 -2*(q.x*q.z - q.w*q.y),
					 q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
					 res);
		break;

	case yzy:
		twoaxisrot(2*(q.y*q.z - q.w*q.x),
			       2*(q.x*q.y + q.w*q.z),
				   q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
				   2*(q.y*q.z + q.w*q.x),
				   -2*(q.x*q.y - q.w*q.z),
				   res);
		break;

	case zyx:
		threeaxisrot(-2*(q.x*q.y - q.w*q.z),
					 q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
					 2*(q.x*q.z + q.w*q.y),
					 -2*(q.y*q.z - q.w*q.x),
					 q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
					 res);
		break;
		 
	case xyx:
		twoaxisrot(2*(q.x*q.y - q.w*q.z),
				   2*(q.x*q.z + q.w*q.y),
				   q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
				   2*(q.x*q.y + q.w*q.z),
				   -2*(q.x*q.z - q.w*q.y),
				   res);
		break;
		 
	case yzx:
		threeaxisrot(2*(q.x*q.z + q.w*q.y),
					 q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
					 -2*(q.x*q.y - q.w*q.z),
					 2*(q.y*q.z + q.w*q.x),
					 q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,
					 res);
		break;
		 
	case xzx:
		twoaxisrot(2*(q.x*q.z + q.w*q.y),
				   -2*(q.x*q.y - q.w*q.z),
				   q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
				   2*(q.x*q.z - q.w*q.y),
				   2*(q.x*q.y + q.w*q.z),
				   res);
		break;
	default:
		std::cout << "Unknown rotation sequence" << std::endl;
		break;
	 }
}

///////////////////////////////
// Helper functions
///////////////////////////////
Quaternion operator*(Quaternion& q1, Quaternion& q2){
	Quaternion q;
	q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
	q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
	q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
	q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
	return q;
}

ostream& operator <<(std::ostream& stream, const Quaternion& q) {
	cout << q.w << " "<< showpos << q.x << "i " << q.y << "j " << q.z << "k"; 
	cout << noshowpos;
}

double rad2deg(double rad){
	return rad*180.0/M_PI;
}

void toAxisSeq(RotSeq seq, int* axis)
{
	switch (seq)
	{
	case xyz:
		axis[0] = 0; axis[1] = 1; axis[2] = 2;
		break;
	case zyz:
		axis[0] = 2; axis[1] = 1; axis[2] = 2;
		break;
	case yxz:
		axis[0] = 1; axis[1] = 0; axis[2] = 2;
		break;
	case zxz:
		axis[0] = 2; axis[1] = 0; axis[2] = 2;
		break;
	case zxy:
		axis[0] = 2; axis[1] = 0; axis[2] = 1;
		break;
	case yxy:
		axis[0] = 1; axis[1] = 0; axis[2] = 1;
		break;
	case xzy:
		axis[0] = 0; axis[1] = 2; axis[2] = 1;
		break;
	case yzy:
		axis[0] = 1; axis[1] = 2; axis[2] = 1;
		break;
	case zyx:
		axis[0] = 2; axis[1] = 1; axis[2] = 0;
		break;
	case xyx:
		axis[0] = 0; axis[1] = 1; axis[2] = 0;
		break;
	case yzx:
		axis[0] = 1; axis[1] = 2; axis[2] = 0;
		break;
	case xzx:
		axis[0] = 0; axis[1] = 2; axis[2] = 0;
		break;
	}
}

Quaternion fromEuler(double rot0, double rot1, double rot2, RotSeq seq)
{
	int iseq[3];
	toAxisSeq(seq, iseq);
	Vec3 axisTable[3] = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1}
	};
	
	double rots[3] = { rot0, rot1, rot2 };
	Vec3 axis[3] = { axisTable[iseq[0]], axisTable[iseq[1]], axisTable[iseq[2]] };
	
	Quaternion q;
	for (int i = 0; i < 3; ++i)
	{ // rot0 around axis0 -> rot1 around axis1 -> rot2 around axis2
		double s = sin(rots[i] / 2);
		double c = cos(rots[i] / 2);
		Quaternion q1(s * axis[i].x, s * axis[i].y, s * axis[i].z, c);
		q = q1 * q;
	}
	
	return q;
}

bool fequal(double d1, double d2, double epsilon = 1e-5)
{
	return abs(d1 - d2) < epsilon;
}

bool testcase(double rot0, double rot1, double rot2, RotSeq seq, bool onlycheck = false)
{
	// 1. construct quaternion from input euler
	Quaternion q = fromEuler(rot0, rot1, rot2, seq);
	
	double euler[3];
	quaternion2Euler(q, euler, seq);
	
	Quaternion q2 = fromEuler(euler[0], euler[1], euler[2], seq);
	
	bool equal = 
		fequal(q.x, q2.x) && fequal(q.y, q2.y) && fequal(q.z, q2.z) && fequal(q.w, q2.w) ||
		fequal(-q.x, q2.x) && fequal(-q.y, q2.y) && fequal(-q.z, q2.z) && fequal(-q.w, q2.w);
	
	bool failed = !equal || !fequal(q.norm(), 1) || !fequal(q2.norm(), 1);
	
	if (!onlycheck || failed)
	{
		int iseq[3];
		toAxisSeq(seq, iseq);
		const char* axisStr[3] = {
			"X", "Y", "Z"
		};

		printf("Rotation sequence: %s(%.2f)->%s(%.2f)->%s(%.2f)\n", 
			axisStr[iseq[0]], rad2deg(rot0), 
			axisStr[iseq[1]], rad2deg(rot1),
			axisStr[iseq[2]], rad2deg(rot2));
	
		cout << "q: " << q << " with length: " << q.norm() << endl;
		cout << "q2: " << q2 << " with length: " << q.norm() << endl;
		printf("Conversion euler: %s(%.2f)->%s(%.2f)->%s(%.2f)\n", 
				axisStr[iseq[0]], rad2deg(euler[0]), 
				axisStr[iseq[1]], rad2deg(euler[1]),
				axisStr[iseq[2]], rad2deg(euler[2]));
		
		if (failed)
		{
			cerr << "testcase failed.\n" << endl;
		}
		else
		{
			cout << endl;
		}
	}
	
	return !failed;
}

///////////////////////////////
// Main
///////////////////////////////
int main()
{
	int failedCount = 0;
	for (int i = 0; i < 1000; ++i)
	{
		double rot0 = (rand() % 10000 / 10000.0) * M_PI * 2;
		double rot1 = (rand() % 10000 / 10000.0) * M_PI * 2;
		double rot2 = (rand() % 10000 / 10000.0) * M_PI * 2;
		RotSeq seq = (RotSeq) (rand() % SeqCount);
		
		if (!testcase(rot0, rot1, rot2, seq, true))
			++failedCount;
	}
	
	printf("failed failedCount: %d\n", failedCount);
	return 0;
}
