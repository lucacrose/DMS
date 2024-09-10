#include <iostream>
#include <fstream>
#include <string>
using namespace std;

const float PI = 3.141592653589;
const float TAU = 2 * PI;

float fmod(float a, float b) {
    return a - (int)(a / b) * b;
}

class Vector3 {
    public:
        float x;
        float y;
        float z;

        Vector3() : x(0), y(0), z(0) {};
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

        Vector3 mod(float n) {
            return Vector3(fmod(x, n), fmod(y, n), fmod(z, n));
        }
};

class Mat3x3 {
    public:
        Vector3 vx;
        Vector3 vy;
        Vector3 vz;

        Mat3x3() : vx(Vector3(1, 0, 0)), vy(Vector3(0, 1, 0)), vz(Vector3(0, 0, 1)) {};
        Mat3x3(Vector3 vx, Vector3 vy, Vector3 vz) : vx(vx), vy(vy), vz(vz) {};

        Mat3x3& operator+=(const Mat3x3& m) {
            this->vx += m.vx;
            this->vy += m.vy;
            this->vz += m.vz;
            return *this;
        }

        Mat3x3 operator*(const float n) const {
            return Mat3x3(vx * n, vy * n, vz * n);
        }

        Mat3x3 cross(const Vector3 v) {
            return Mat3x3(vx.cross(v), vy.cross(v), vz.cross(v));
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
        Mat3x3 theta; // rotation matrix

        Drone(float mass, float inertia) : invMass(1 / mass), invInertia(1 / inertia) {};

        void applyForce(Vector3 force, Vector3 radius) { // apply force (N) at radius (m)
            Vector3 torque = radius.cross(force);

            alpha = (alpha + torque * invInertia).mod(TAU);
            acceleration += force * invMass;
        }

        // TODO: Better integrator?
        void physicsStep(float t) { // run a physics step with a fidelity of t (s)
            velocity += acceleration * t;
            omega = (omega + alpha * t).mod(TAU);

            position += velocity * t;
            theta += theta.cross(omega) * t;

            acceleration = Vector3();
            alpha = Vector3();
        }
};

int main() {
    Drone d(1, 1);

    d.applyForce(Vector3(0, 1, 0), Vector3(1, 0, 1));

    string out = "return \"";
    
    for (int i = 0; i < 1000; i++) {
        d.physicsStep(.1);
        out += std::to_string(d.position.x) + "," + std::to_string(d.position.y) + "," + std::to_string(d.position.z) + "," + std::to_string(d.theta.vx.x) + "," + std::to_string(d.theta.vx.y) + "," + std::to_string(d.theta.vx.z) + "," + std::to_string(d.theta.vy.x) + "," + std::to_string(d.theta.vy.y) + "," + std::to_string(d.theta.vy.z) + "," + std::to_string(d.theta.vz.x) + "," + std::to_string(d.theta.vz.y) + "," + std::to_string(d.theta.vz.z);

        if (i < 999) {
            out += "|";
        } else {
            out += "\"";
        }
    }

    ofstream MyFile("output.txt");

    MyFile << out;

    MyFile.close();

    return 0;
}