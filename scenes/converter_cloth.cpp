#include "converter_general.h"
#include "MitsubaGenerator.h"

using namespace std;
using namespace chrono;
using namespace chrono::collision;

int main(int argc, char* argv[]) {
  if (argc == 1) {
    cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << endl;
    MitsubaGenerator scene_document("scene.xml");
    scene_document.camera_origin = ChVector<>(.2, .65, -.35) * 100;
    scene_document.camera_target = ChVector<>(-.4, .4, .4) * 100;
    scene_document.scale = 3;
    scene_document.turbidity = 10;
    scene_document.CreateScene();
    scene_document.AddShape("background", ChVector<>(1), ChVector<>(0),
                            ChQuaternion<>(1, 0, 0, 0));
    scene_document.Write();
    return 0;
  }
  stringstream input_file_ss;
  input_file_ss << argv[1] << "_state.txt";
  string data;
  ReadCompressed(input_file_ss.str(), data);
  std::replace(data.begin(), data.end(), ',', '\t');
  stringstream output_file_ss;
  if (argc == 3) {
    output_file_ss << argv[2] << argv[1] << ".xml";
  } else {
    output_file_ss << argv[1] << ".xml";
  }
  MitsubaGenerator data_document(output_file_ss.str());
  stringstream data_stream(data);
  ChQuaternion<> rot;
  ChVector<> pos, vel, scale;
  int counter = 0;

  SkipLine(data_stream, 6);

  //  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  //  scale.y = scale.z = scale.x;
  //  data_document.AddShape("sphere", scale, pos, rot);
  int rings = 0;

  while (data_stream.fail() == false) {
    // int type = ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    ProcessLine(data_stream, pos, vel, rot);

    if (data_stream.fail() == false) {
      data_document.AddShape("ring", ChVector<>(1, 1, 1), pos, rot);
      // SkipLine(data_stream, 15);
      rings++;
    }
  }
  std::cout << "total rings: " << rings << std::endl;

  data_document.Write();
  return 0;
}
