#include "converter_general.h"
#include "MitsubaGenerator.h"

using namespace std;
using namespace chrono;
using namespace chrono::collision;

int main(int argc, char* argv[]) {
  if (argc == 1) {
    cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << endl;
    MitsubaGenerator scene_document;
    scene_document.camera_up = ChVector<>(0, -1, 0);
    scene_document.camera_origin = ChVector<>(0, 0, -1);
    scene_document.camera_target = ChVector<>(0, 0, 0);
    scene_document.scale = 3;
    scene_document.turbidity = 10;
    scene_document.CreateScene();
    scene_document.AddShape("background", ChVector<>(1), ChVector<>(0), ChQuaternion<>(1, 0, 0, 0));
    scene_document.Write("scene.xml");
    return 0;
  }
  stringstream input_file_ss;
  input_file_ss << argv[1] << "_state.txt";

  string data;
  ReadCompressed(input_file_ss.str(), data);
  std::replace(data.begin(), data.end(), ',', '\t');

  MitsubaGenerator data_document;
  stringstream data_stream(data);
  ChQuaternion<> rot;
  ChVector<> pos, vel, scale;
  int counter = 0;
  // std::cout << data << std::endl;

  //  0.01,0.1724,0.147,
  //  0.01,0.1724,0.147,
  //  0.147,0.1724,0.01,
  //  0.147,0.1724,0.01,
  //  ,,0,0,0,2,0.635,0.01,0.635,
  //  ,,0,0,0,2,0.635,0.01,0.635,
  //  ,0.382683,0.92388,0,0 ,0,0,0 ,2,0.635,0.01,0.635,
  //  ,-0.382683,0.92388,0,0,0,0,0,2, 0.635,0.01,0.635,
  //
  //
  data_document.AddShape("cube", ChVector<>(0.147, 0.01, 0.147), ChVector<>(0, 0.6958, -1.22465e-18),
                         ChQuaternion<>(6.12323e-17, 1, 0, 0));
  //  data_document.AddShape("cube", ChVector<>(0.01, 0.1724, 0.147), ChVector<>(-0.137, 0.5334, 1.86636e-17),
  //                         ChQuaternion<>(6.12323e-17, 1, 0, 0));
  data_document.AddShape("cube", ChVector<>(0.01, 0.1724, 0.147), ChVector<>(0.137, 0.5334, 1.86636e-17),
                         ChQuaternion<>(6.12323e-17, 1, 0, 0));
  data_document.AddShape("cube", ChVector<>(0.147, 0.1724, 0.01), ChVector<>(0, 0.5334, 0.137),
                         ChQuaternion<>(6.12323e-17, 1, 0, 0));
  data_document.AddShape("cube", ChVector<>(0.147, 0.1724, 0.01), ChVector<>(0, 0.5334, -0.137),
                         ChQuaternion<>(6.12323e-17, 1, 0, 0));
  //  data_document.AddShape("cube", ChVector<>(0.635, 0.01, 0.635), ChVector<>(0.587803, -0.0635, 3.88825e-17),
  //                         ChQuaternion<>(5.65713e-17, 0.92388, -0.382683, 2.34326e-17));
  //  data_document.AddShape("cube", ChVector<>(0.635, 0.01, 0.635), ChVector<>(-0.587803, -0.0635, 3.88825e-17),
  //                         ChQuaternion<>(5.65713e-17, 0.92388, 0.382683, -2.34326e-17));
  data_document.AddShape("cube", ChVector<>(0.635, 0.01, 0.635), ChVector<>(0, -0.0635, -0.587803),
                         ChQuaternion<>(0.382683, 0.92388, 0, 0));
  data_document.AddShape("cube", ChVector<>(0.635, 0.01, 0.635), ChVector<>(0, -0.0635, 0.587803),
                         ChQuaternion<>(-0.382683, 0.92388, 0, 0));

  //  ofstream ofile_rings("rings.txt");
  //  ofstream ofile_clasp("clasps.txt");

  // SkipLine(data_stream, 9);
  SkipLine(data_stream, 2);
  // SkipLine(data_stream, 40760);
  // the box that presses down
  int type = ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("box", scale, pos, rot);

  for (int i = 0; i < 40760; i++) {
    ProcessLine(data_stream, pos, vel, rot);
    if (data_stream.fail() == false) {
      data_document.AddShape("ring", ChVector<>(1), pos, rot);
      //      ofile_rings << pos.x << " " << pos.y << " " << pos.z << " " << rot.e0 << " " << rot.e1 << " " << rot.e2 <<
      //      " "
      //                  << rot.e3 << std::endl;
    }
  }
  for (int i = 0; i < 55; i++) {
    ProcessLine(data_stream, pos, vel, rot);
    if (data_stream.fail() == false) {
      data_document.AddShape("clasp", ChVector<>(1), pos, rot);
      //      ofile_clasp << pos.x << " " << pos.y << " " << pos.z << " " << rot.e0 << " " << rot.e1 << " " << rot.e2 <<
      //      " "
      //                  << rot.e3 << std::endl;
    }
  }
  //  ofile_rings.close();
  //  ofile_clasp.close();

  stringstream output_file_ss;
  if (argc == 3) {
    output_file_ss << argv[2] << argv[1] << ".xml";
  } else {
    output_file_ss << argv[1] << ".xml";
  }
  data_document.Write(output_file_ss.str());
  return 0;
}
