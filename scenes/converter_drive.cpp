#include "converter_general.h"
#include "MitsubaGenerator.h"

using namespace std;
using namespace chrono;

int main(int argc, char *argv[]) {
  if (argc == 1) {
    cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << endl;
    MitsubaGenerator scene_document("scene.xml");
    scene_document.camera_origin = ChVector<>(0, -10, 0);
    scene_document.camera_target = ChVector<>(0, 0, 0);
    scene_document.camera_up = ChVector<>(0, 0, 1);
    scene_document.scale = 3;
    scene_document.turbidity = 10;
    scene_document.CreateScene();
    // scene_document.AddShape("background", ChVector<>(1), ChVector<>(0, -10,
    // 0), ChQuaternion<>(1, 0, 0, 0));
    scene_document.Write();
    return 0;
  }
  stringstream input_file_ss, input_file_vehicle;
  input_file_ss << "data_" << argv[1] << ".dat";
  input_file_vehicle << "vehicle_" << argv[1] << ".dat";
  std::ifstream ifile;
  OpenBinary(input_file_ss.str(), ifile);

  std::vector<half3> position;
  std::vector<half3> velocity;

  ReadBinary(ifile, position);
  CloseBinary(ifile);

  string data_v;
  ReadCompressed(input_file_vehicle.str(), data_v);
  std::replace(data_v.begin(), data_v.end(), ',', '\t');
  stringstream output_file_ss;
  if (argc == 3) {
    output_file_ss << argv[2] << argv[1] << ".xml";
  } else {
    output_file_ss << argv[1] << ".xml";
  }
  MitsubaGenerator data_document(output_file_ss.str());

  stringstream vehicle_stream(data_v);

  ChVector<> pos, vel, scale;
  ChQuaternion<> rot;
  int count = 0;

  for (int i = 0; i < position.size(); i++) {
    pos.x = position[i].x;
    pos.y = position[i].y;
    pos.z = position[i].z;

    // double v = vel.Length() * 2;

    data_document.AddShape("sphere", .1, pos, QUNIT);
    //      data_document.AddCompleteShape("sphere", "diffuse", VelToColor(v),
    //      .015,
    //                                     pos, QUNIT);

    count++;
  }
  // std::cout << data_v << std::endl;

  ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
  chrono::ChQuaternion<> q;
  q.Q_from_AngAxis(CH_C_PI, chrono::ChVector<>(1, 0, 0));
  rot = rot * q;
  Vector offset = rot.Rotate(Vector(-0.055765, 0, -0.52349));
  data_document.AddShape("chassis", Vector(1, 1, 1), pos + offset, rot);

  Vector camera_pos = pos + offset;
  camera_pos.z = 4;
  camera_pos.y -= 8;
  camera_pos.x += 0;
  data_document.AddSensor(camera_pos, pos + offset, Vector(0, 0, 1));

  ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel_R", Vector(1, 1, 1), pos, rot);
  ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel_R", Vector(1, -1, 1), pos, rot);
  ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel_R", Vector(1, 1, 1), pos, rot);
  ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel_R", Vector(1, -1, 1), pos, rot);

  data_document.Write();
  return 0;
}
