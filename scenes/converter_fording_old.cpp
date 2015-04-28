#include "converter_general.h"
#include "chrono_utils/ChUtilsMitsuba.h"
#include "collision/ChCCollisionSystemBullet.h"

using namespace std;
using namespace chrono;
using namespace chrono::utils;
using namespace chrono::collision;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    cout << "REQURES FRAME NUMBER AS ARGUMENT" << endl;
    return 1;
  }
  stringstream input_file_ss;
  input_file_ss << argv[1] << ".txt";


  string data;
  ReadCompressed(input_file_ss.str(), data);
  std::replace(data.begin(), data.end(), ',', '\t');


  ChMitsubaRender scene_document;
  scene_document.CreateScene();
  scene_document.Write("scene.xml");
  ChMitsubaRender data_document;

  stringstream data_stream(data);

  SkipLine(data_stream, 12);

  ChVector<> pos, vel, scale;
  ChQuaternion<> rot;
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("chassis", scale, pos, rot);
  SkipLine(data_stream, 2);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", scale, pos, rot);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", scale, pos, rot);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", scale, pos, rot);
  ProcessPovrayLine(data_stream, pos, vel, scale, rot);
  data_document.AddShape("wheel", scale, pos, rot);
  while (data_stream.fail() == false) {
    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    if (data_stream.fail() == false) {
      data_document.AddShape("sphere", ChVector<>(0.011), pos, rot);
    }
  }
  stringstream output_file_ss;
  output_file_ss << argv[1] << ".xml";
  data_document.Write(output_file_ss.str());
  return 0;
}
