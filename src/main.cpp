#include <iostream>
using namespace std;

class Vector3 {
    public:
        float x;
        float y;
        float z;

        Vector3() {};
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {};

        friend ostream& operator<<(ostream& os, const Vector3& v) {
            os << "Vector3 (" << v.x << ", " << v.y << ", " << v.z << ")";
            return os;
        }

        Vector3 operator+(const Vector3& v) const {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }
};

class Drone {
    public:
        float mass; // kg
        Vector3 position; // m
        Vector3 orientation; // rad; yaw, pitch, roll

        void applyTorque(float f, Vector3 r, float t) { // apply torque of f newtons at radius vector r for t seconds

        }
};

int main() {
    Drone d;

    d.applyTorque(1, Vector3(0,0,0), 1);

    std::cout << "Position: " << d.position << "\nOrientation: " << d.orientation;
    return 0;
}