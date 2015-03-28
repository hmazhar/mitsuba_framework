#include "converter_general.h"
#include "MitsubaGenerator.h"

using namespace std;
using namespace chrono;

int main(int argc, char* argv[]) {
  if (argc == 1) {
    cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << endl;
    MitsubaGenerator scene_document;
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

  gzFile gz_file = gzopen(input_file_ss.str().c_str(), "rb");
  unsigned long int size;
  gzread(gz_file, (void*)&size, sizeof(size));
  std::string data;
  data.resize(size / sizeof(char));
  gzread(gz_file, (void*)data.data(), size);
  gzclose(gz_file);
  MitsubaGenerator data_document;

  stringstream data_stream(data);
  SkipLine(data_stream, 5);

  ChVector<> pos, vel, scale;
  ChQuaternion<> rot;

  int type = ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("base", scale, pos, rot);

  while (data_stream.fail() == false) {
    int type = ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    if (data_stream.fail() == false) {
      switch (type) {
        case chrono::collision::SPHERE:
          data_document.AddShape("sphere", scale, pos, rot);
          break;
        case chrono::collision::ELLIPSOID:
          data_document.AddShape("ellipsoid", scale, pos, rot);
          break;
        case chrono::collision::BOX:
          data_document.AddShape("box", scale, pos, rot);
          break;
        case chrono::collision::CYLINDER:
          data_document.AddShape("cylinder", scale, pos, rot);
          break;
        case chrono::collision::CONE:
          data_document.AddShape("cone", scale, pos, rot);
          break;
        default:
          // type is -1 (triangle mesh)
          break;
      }
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
