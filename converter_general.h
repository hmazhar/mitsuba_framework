#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <zlib.h>
#include <core/ChMath.h>
#include <collision/ChCModelBullet.h>

#include "half/half.hpp"
class half2 {
  public:
    half2() : x(0.0f), y(0.0f) {}

    half_float::half x, y;
};

class half3 {
  public:
    half3() : x(0.0f), y(0.0f), z(0.0) {}

    half_float::half x, y, z;
};

class half4 {
  public:
    half4() : x(0.0f), y(0.0f), z(0.0), w(0.0) {}

    half_float::half x, y, z, w;
};

void ReadCompressed(std::string filename, std::string& data) {
    gzFile gz_file = gzopen(filename.c_str(), "rb");
    unsigned long int size;
    gzread(gz_file, (void*)&size, sizeof(size));
    data.resize(size / sizeof(char));

    gzread(gz_file, (void*)data.data(), size);
    gzclose(gz_file);
}

void OpenBinary(std::string filename, std::ifstream& bin_file) {
    bin_file.open(filename.c_str(), std::ios::in | std::ofstream::binary);
    if (bin_file.is_open()) {
        printf("File successfully open\n");
    } else {
        printf("Error opening %s\n", filename.c_str());
    }
}

template <typename T>
void ReadBinary(std::ifstream& bin_file, std::vector<T>& data) {
    size_t size = 0;
    bin_file.read(reinterpret_cast<char*>(&size), sizeof(size));
    data.resize(size);
    bin_file.read(reinterpret_cast<char*>(&data[0]), size * sizeof(T));
}
void CloseBinary(std::ifstream& bin_file) {
    bin_file.close();
}

void SkipLine(std::stringstream& ifile, int number = 1) {
    std::string temp;
    for (int i = 0; i < number; i++) {
        getline(ifile, temp);
    }
}
void ProcessLine(std::stringstream& ifile, chrono::ChVector<>& pos, chrono::ChVector<>& vel, chrono::ChQuaternion<>& quat) {
    std::string temp;
    std::getline(ifile, temp);
    std::stringstream ss(temp);
    double junk;
    ss >> pos.x >> pos.y >> pos.z;
    ss >> quat.e0 >> quat.e1 >> quat.e2 >> quat.e3;
    ss >> vel.x >> vel.y >> vel.z;
    ss >> junk >> junk >> junk;
}
int ProcessPovrayLine(std::stringstream& ifile, chrono::ChVector<>& pos, chrono::ChVector<>& vel, chrono::ChVector<>& rad, chrono::ChQuaternion<>& quat) {
    std::string temp;
    std::getline(ifile, temp);
    std::stringstream ss(temp);
    int type;
    rad = chrono::ChVector<>(1, 1, 1);
    ss >> pos.x >> pos.y >> pos.z;
    ss >> quat.e0 >> quat.e1 >> quat.e2 >> quat.e3;
    ss >> vel.x >> vel.y >> vel.z;
    ss >> type;
    switch (type) {
        case chrono::collision::SPHERE:
            ss >> rad.x;
            rad.y = rad.z = rad.x;
            break;
        case chrono::collision::ELLIPSOID:
            ss >> rad.x >> rad.y >> rad.z;
            break;
        case chrono::collision::BOX:
            ss >> rad.x >> rad.y >> rad.z;
            break;
        case chrono::collision::CYLINDER:
            ss >> rad.x >> rad.y;
            rad.z = rad.x;
            break;
        case chrono::collision::CONE:
            ss >> rad.x >> rad.y;
            rad.z = rad.x;
            break;
        case chrono::collision::CAPSULE:
            ss >> rad.x >> rad.y;
            rad.z = rad.x;
            break;
        default:
            // type is -1 (triangle mesh)
            return -1;
            break;
    }
    return type;
}

void ProcessPosVel(std::stringstream& ifile, chrono::ChVector<>& pos, chrono::ChVector<>& vel) {
    std::string temp;
    std::getline(ifile, temp);
    std::stringstream ss(temp);
    ss >> pos.x >> pos.y >> pos.z;
    ss >> vel.x >> vel.y >> vel.z;
}

// velocity must be scaled from 0-1;
chrono::ChVector<> VelToColor(double v) {
    chrono::ChVector<> c(1, 1, 1);

    if (v <= 0.5) {
        c = (chrono::ChVector<>(0, 1, 0) * v * 2.0 + chrono::ChVector<>(0, 0, 1) * (.5 - v) * 2.0);

    } else if (v > 0.5 && v < 1.0) {
        c = (chrono::ChVector<>(1, 0, 0) * (v - .5) * 2.0 + chrono::ChVector<>(0, 1, 0) * (1.0 - v) * 2.0);
    } else {
        c = chrono::ChVector<>(1, 0, 0);
    }
    return c;
}
