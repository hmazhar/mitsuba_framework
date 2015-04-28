#include "converter_general.h"
#include "chrono_utils/ChUtilsMitsuba.h"

using namespace std;
using namespace chrono;
using namespace chrono::utils;
using namespace chrono::collision;

int main(int argc, char* argv[]) {
  if (argc == 1) {
    cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << endl;
    ChMitsubaRender scene_document;
    scene_document.camera_origin = ChVector<>(0, .75, -2);
    scene_document.camera_target = ChVector<>(0, .5, -1);
    scene_document.scale = 3;
    scene_document.turbidity = 10;
    scene_document.CreateScene();
    scene_document.AddShape("background", ChVector<>(1), ChVector<>(0), ChQuaternion<>(1, 0, 0, 0));
    scene_document.Write("scene.xml");
    return 0;
  }
  stringstream input_file_ss;
  input_file_ss << argv[1] << ".txt";


  string data;
  ReadCompressed(input_file_ss.str(), data);
  std::replace(data.begin(), data.end(), ',', '\t');

  ChMitsubaRender data_document;
  stringstream data_stream(data);
  SkipLine(data_stream, 5);
  ChVector<> pos, vel, scale;
  ChQuaternion<> rot;
  SkipLine(data_stream, 121167);
  SkipLine(data_stream, 213744);
//  for(int i=0; i<121167; i++) {
//    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
//    if (data_stream.fail() == false) {
//      data_document.AddShape("sphere_out", ChVector<>(.015 * 2), pos, rot);
//    }
//  }
//  for(int i=0; i<213744; i++) {
//    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
//    if (data_stream.fail() == false) {
//      data_document.AddShape("sphere_in", ChVector<>(.015 * 2), pos, rot);
//    }
//  }

  while (data_stream.fail() == false) {
    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    data_document.AddShape("heart", ChVector<>(1), pos, rot);
    SkipLine(data_stream, 9);
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
