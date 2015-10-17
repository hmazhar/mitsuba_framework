#include "converter_general.h"
#include "chrono_utils/ChUtilsMitsuba.h"
#include "collision/ChCCollisionSystemBullet.h"

using namespace std;
using namespace chrono;
using namespace chrono::utils;
using namespace chrono::collision;

int main(int argc, char* argv[]) {
  if (argc == 1) {
    cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << endl;
    MitsubaGenerator scene_document("scene.xml");
    scene_document.camera_origin = ChVector<>(4, 0, 0);
    scene_document.camera_target = ChVector<>(3, 0, 0);
    scene_document.scale = 3;
    scene_document.turbidity = 10;
    scene_document.CreateScene();
    scene_document.AddShape(
        "background", ChVector<>(1), ChVector<>(0), ChQuaternion<>(1, 0, 0, 0));
    scene_document.Write();
    return 0;
  }
  stringstream input_file_ss;
  input_file_ss << argv[1] << ".txt";

  string data;
  ReadCompressed(input_file_ss.str(), data);
  std::replace(data.begin(), data.end(), ',', '\t');
  stringstream output_file_ss;
   if (argc == 3) {
     output_file_ss << argv[2] << argv[1] << ".xml";
   } else {
     output_file_ss << argv[1] << ".xml";
   }

  ChMitsubaRender data_document(output_file_ss.str());
  stringstream data_stream(data);
  SkipLine(data_stream, 6);
  ChVector<> pos, vel, scale;
  ChQuaternion<> rot;
  while (data_stream.fail() == false) {
    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    if (data_stream.fail() == false) {
      data_document.AddShape("sphere", ChVector<>(0.005), pos, rot);
    }
  }

  data_document.Write();
  return 0;
}
