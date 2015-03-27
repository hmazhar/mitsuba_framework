#include "converter_general.h"
#include "MitsubaGenerator.h"

using namespace std;
using namespace chrono;
using namespace chrono::collision;

int main(int argc, char* argv[]) {
  if (argc == 1) {
    cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << endl;
    MitsubaGenerator scene_document;
    scene_document.camera_origin = ChVector<>(.2, .65, -.35) * 100;
    scene_document.camera_target = ChVector<>(-.4, .4, .4) * 100;
    scene_document.scale = 3;
    scene_document.turbidity = 10;
    scene_document.CreateScene();
    scene_document.AddShape("background", ChVector<>(1), ChVector<>(0), ChQuaternion<>(1, 0, 0, 0));
    scene_document.Write("scene.xml");
    return 0;
  }
  stringstream input_file_ss;
  input_file_ss << argv[1] << ".txt";

  gzFile gz_file = gzopen(input_file_ss.str().c_str(), "rb");
  unsigned long int size;
  gzread(gz_file, (void*)&size, sizeof(size));
  std::string data;
  data.resize(size / sizeof(char));
  gzread(gz_file, (void*)data.data(), size);
  gzclose(gz_file);
  MitsubaGenerator data_document;
  stringstream data_stream(data);
  ChQuaternion<> rot;
  ChVector<> pos, vel, scale;
  int counter = 0;

  SkipLine(data_stream, 6);

//  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
//  scale.y = scale.z = scale.x;
//  data_document.AddShape("sphere", scale, pos, rot);

  while (data_stream.fail() == false) {
    int type = ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    if (data_stream.fail() == false) {
      data_document.AddShape("ring", scale, pos, rot);
      //SkipLine(data_stream, 15);
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
