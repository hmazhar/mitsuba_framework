#include "converter_general.h"
#include "MitsubaGenerator.h"
//#include "Partio.h"
using namespace std;
using namespace chrono;

int main(int argc, char* argv[]) {
  if (argc == 1) {
    cout << "REQURES FRAME NUMBER AS ARGUMENT" << endl;
    return 1;
  }
  stringstream input_file_ss;
  input_file_ss << "data_" << argv[1] << ".dat";
  string data;
  ReadCompressed(input_file_ss.str(), data);
  //std::cout<<data<<std::endl;
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
  camera_pos.y = 6;
  // mo
  camera_pos.z += 1;
  camera_pos.x += 3;
  data_document.AddSensor(camera_pos, pos + offset, Vector(0, 0, 1));

  SkipLine(data_stream, 5);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", Vector(1, 1, 1), pos, rot);

  SkipLine(data_stream, 22);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", Vector(1, 1, 1), pos, rot);
  SkipLine(data_stream, 22);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", Vector(1, 1, 1), pos, rot);
  SkipLine(data_stream, 22);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", Vector(1, 1, 1), pos, rot);
  SkipLine(data_stream, 22);

  //  data_document.AddShape("fluid", ChVector<>(1), ChVector<>(0), ChQuaternion<>(1, 0, 0, 0));
  //  //
  //  Partio::ParticlesDataMutable& data_p = *Partio::create();
  //  Partio::ParticleAttribute positionAttr = data_p.addAttribute("position", Partio::VECTOR, 3);
  //  Partio::ParticleAttribute velocityAttr = data_p.addAttribute("v", Partio::VECTOR, 3);
  //
  //  while (data_stream.fail() == false) {
  //    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  //    if (data_stream.fail() == false) {
  //      Partio::ParticleIndex index = data_p.addParticle();
  //      float* pos_partio = data_p.dataWrite<float>(positionAttr, index);
  //      pos_partio[0] = pos.x;
  //      pos_partio[1] = pos.y;
  //      pos_partio[2] = pos.z;
  //      float* vel_partio = data_p.dataWrite<float>(velocityAttr, index);
  //      vel_partio[0] = vel.x;
  //      vel_partio[1] = vel.y;
  //      vel_partio[2] = vel.z;
  //      //      //std::cout<<vel.Length()<<std::endl;
  //      // data_document.AddShape("sphere", ChVector<>(0.03), pos, rot);
  //    }
  //  }
  //  //
  //  stringstream partio_ss;
  //  partio_ss << argv[1] << ".bgeo";
  //  Partio::write(partio_ss.str().c_str(), data_p);

  while (data_stream.fail() == false) {
    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    if (data_stream.fail() == false) {
      data_document.AddShape("sphere", ChVector<>(0.0175), pos, rot);
    }
  }
  stringstream output_file_ss;

  if (argc == 3) {
    output_file_ss << argv[2] << argv[1] << ".xml";
  } else {
    output_file_ss << argv[1] << ".xml";
  }
  data_document.Write(output_file_ss.str());
  return 0;
}
