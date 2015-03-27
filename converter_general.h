#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <zlib.h>
#include <core/ChMath.h>
#include <collision/ChCModelBullet.h>

void ReadCompressed(std::string filename, std::string& data) {
  gzFile gz_file = gzopen(filename.c_str(), "rb");
  unsigned long int size;
  gzread(gz_file, (void*)&size, sizeof(size));
  data.resize(size / sizeof(char));

  gzread(gz_file, (void*)data.data(), size);
  gzclose(gz_file);
}

void SkipLine(std::stringstream& ifile, int number = 1) {
  std::string temp;
  for (int i = 0; i < number; i++) {
    getline(ifile, temp);
  }
}

int ProcessPovrayLine(std::stringstream& ifile,
                      chrono::ChVector<>& pos,
                      chrono::ChVector<>& vel,
                      chrono::ChVector<>& rad,
                      chrono::ChQuaternion<>& quat) {
  std::string temp;
  std::getline(ifile, temp);
  std::replace(temp.begin(), temp.end(), ',', '\t');
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
    default:
      // type is -1 (triangle mesh)
      break;
  }
  return type;
}
