#include "converter_general.h"
#include "MitsubaGenerator.h"

using namespace std;
using namespace chrono;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    cout << "REQURES FRAME NUMBER AS ARGUMENT" << endl;
    return 1;
  }
  stringstream input_file_ss;
  input_file_ss << "data_" << argv[1] << ".dat";

  gzFile gz_file = gzopen(input_file_ss.str().c_str(), "rb");
  unsigned long int size;
  gzread(gz_file, (void*)&size, sizeof(size));
  std::string data;
  data.resize(size / sizeof(char));

  gzread(gz_file, (void*)data.data(), size);
  gzclose(gz_file);

  MitsubaGenerator scene_document;
  scene_document.CreateScene(true, false);
  scene_document.Write("scene.xml");
  MitsubaGenerator data_document;
  stringstream data_stream(data);




  ChVector<> pos, vel, scale;
  ChQuaternion<> rot;
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("box", scale, pos, rot);
  SkipLine(data_stream, 3);
//  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
//  data_document.AddShape("box", scale, pos, rot);
//  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
//  data_document.AddShape("box", scale, pos, rot);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("box", scale, pos, rot);
  SkipLine(data_stream, 1);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("box", scale, pos, rot);
  SkipLine(data_stream, 1);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("box", scale, pos, rot);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("box", scale, pos, rot);



  ProcessPovrayLine(data_stream, pos, vel, scale, rot);

  Vector offset = rot.Rotate(Vector(-0.055765, 0, -0.52349));

  data_document.AddShape("chassis", Vector(1, 1, 1), pos + offset, rot);
  Vector camera_pos = pos + offset;
  camera_pos.y = 10;
  data_document.AddSensor(camera_pos, pos + offset, Vector(0,0,1));

  SkipLine(data_stream, 256);
  SkipLine(data_stream, 4);

  SkipLine(data_stream, 525);

  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", Vector(1, 1, 1), pos, rot);

  SkipLine(data_stream, 7);
  SkipLine(data_stream, 525);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", Vector(1, 1, 1), pos, rot);
  SkipLine(data_stream, 7);
  SkipLine(data_stream, 525);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", Vector(1, 1, 1), pos, rot);
  SkipLine(data_stream, 7);
  SkipLine(data_stream, 525);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", Vector(1, 1, 1), pos, rot);

  //  while (data_stream.fail() == false) {
  //    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  //    if (data_stream.fail() == false) {
  //      data_document.AddShape("sphere", ChVector<>(0.011), pos, rot);
  //    }
  //  }
  stringstream output_file_ss;
  output_file_ss << argv[1] << ".xml";
  data_document.Write(output_file_ss.str());
  return 0;
}
