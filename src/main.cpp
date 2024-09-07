#include <iostream>
using namespace std;

class Vector3 {
    public:
        float x = 0;
        float y = 0;
        float z = 0;

        Vector3() {};
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {};

        friend ostream& operator<<(ostream& os, const Vector3& v) {
            os << "Vector3 (" << v.x << ", " << v.y << ", " << v.z << ")";
            return os;
        }

        Vector3 operator+(const Vector3& v) const {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        Vector3& operator+=(const Vector3& v) {
            *this = *this + v;
            return *this;
        }

        Vector3 operator*(const float n) const {
            return Vector3(x * n, y * n, z * n);
        }

        Vector3 cross(const Vector3 v) {
            return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
        }
};

class Drone {
    public:
        float invMass; // kg (inverted)
        float invInertia; // kg/m^2 (inverted)

        Vector3 acceleration; // m/s^2
        Vector3 alpha; // rad/s^2

        Vector3 velocity; // m/s
        Vector3 omega; // rad/s

        Vector3 position; // m
        Vector3 theta; // rad

        Drone(float m, float i) : invMass(1 / m), invInertia(1 / i) {};

        void applyForce(Vector3 force, Vector3 radius) { // apply force (N) at radius (m)
            Vector3 torque = force.cross(radius);

            alpha += torque * invInertia;
            acceleration += force * invMass;
        }

        // TODO: Heun's method?
        void physicsStep(float t) { // run a physics step with a fidelity of t (s)
            velocity += acceleration * t;
            omega += alpha * t;

            position += velocity * t;
            theta += omega * t;

            acceleration = Vector3();
            alpha = Vector3();
        }
};

int main() {
    Drone d(1, 1);

    d.applyForce(Vector3(0, 1, 0), Vector3(1, 0, 0));
    
    d.physicsStep(1);

    std::cout << "Position: " << d.position << "\nRotational Position: " << d.theta << "\nLinear Velocity: " << d.velocity << "\nRotational Velocity: " << d.omega << "\nLinear Acceleration: " << d.acceleration << "\nRotational Acceleration: " << d.alpha;
    return 0;
}