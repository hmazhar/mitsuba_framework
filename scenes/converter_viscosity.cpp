#include "converter_general.h"
#include "chrono_utils/ChUtilsMitsuba.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "Partio.h"

using namespace std;
using namespace chrono;
using namespace chrono::utils;
using namespace chrono::collision;


using namespace Partio;

Partio::ParticlesDataMutable *data_p;
Partio::ParticleAttribute pHandle;
Partio::ParticleAttribute vHandle;
Partio::ParticleAttribute aHandle;
string output_filename, delimiter = " ";


int main(int argc, char* argv[]) {
  if (argc == 1) {
    cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << endl;
    ChMitsubaRender scene_document;
    scene_document.camera_origin = ChVector<>(4, 0, 0);
    scene_document.camera_target = ChVector<>(3, 0, 0);
    scene_document.scale = 3;
    scene_document.turbidity = 10;
    scene_document.CreateScene();
    scene_document.AddShape("background", ChVector<>(1), ChVector<>(0), ChQuaternion<>(1, 0, 0, 0));
    scene_document.Write("scene.xml");
    return 0;
  }

  Partio::ParticlesDataMutable& data_p=*Partio::create();
   Partio::ParticleAttribute positionAttr=data_p.addAttribute("position",Partio::VECTOR,3);


  stringstream input_file_ss;
  input_file_ss << argv[1] << ".txt";

  string data;
  ReadCompressed(input_file_ss.str(), data);
  std::replace(data.begin(), data.end(), ',', '\t');

  ChMitsubaRender data_document;
  stringstream data_stream(data);
  SkipLine(data_stream, 6);
  ChVector<> pos, vel, scale;
  ChQuaternion<> rot;

  for (int i = 0; i < 4; i++) {
    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    data_document.AddShape("bucky", ChVector<>(1), pos, rot);
    SkipLine(data_stream, 255);
    //ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    //data_document.AddShape("cube", scale, pos, rot);
    //SkipLine(data_stream, 4);
  }

  while (data_stream.fail() == false) {
    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    if (data_stream.fail() == false) {

    	Partio::ParticleIndex index=data_p.addParticle();
    	float* pos_partio=data_p.dataWrite<float>(positionAttr,index);
    	pos_partio[0] = pos.x;
    	pos_partio[1] = pos.y;
    	pos_partio[2] = pos.z;

      //data_document.AddShape("sphere", ChVector<>(0.005), pos, rot);
    }
  }
  data_document.AddShape("fluid", ChVector<>(1), ChVector<>(0), ChQuaternion<>(1,0,0,0));
  stringstream output_file_ss;
  if (argc == 3) {
    output_file_ss << argv[2] << argv[1] << ".xml";
  } else {
    output_file_ss << argv[1] << ".xml";
  }
  data_document.Write(output_file_ss.str());
  stringstream partio_ss;
  partio_ss << argv[1] << ".bgeo";
  Partio::write(partio_ss.str().c_str(),data_p);

  return 0;
}
