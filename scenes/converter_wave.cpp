#include "converter_general.h"
#include "MitsubaGenerator.h"

using namespace std;
using namespace chrono;

int main(int argc, char* argv[]) {
    if (argc == 1) {
        cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << endl;
        MitsubaGenerator scene_document;
        scene_document.camera_origin = ChVector<>(-80, 0, -80);
        scene_document.camera_target = ChVector<>(0, 0, 0);
        scene_document.scale = 3;
        scene_document.turbidity = 10;
        scene_document.CreateScene();
        scene_document.AddShape("background", ChVector<>(1), ChVector<>(0,-10,0), ChQuaternion<>(1, 0, 0, 0));
        scene_document.Write("scene.xml");
        return 0;
    }
    std::cout<<"start\n";
    stringstream input_file_ss;
    input_file_ss << argv[1] << ".dat";

    string data;
    std::cout<<"read compressed... ";
    ReadCompressed(input_file_ss.str(), data);
    std::cout<<"reading done\n";
    std::cout<<"replacing \n";

    std::replace(data.begin(), data.end(), ',', '\t');

    MitsubaGenerator data_document;
    std::cout<<"streaming \n";
    stringstream data_stream(data);
    data.clear();

    ChVector<> pos, vel;
    int count = 0;
    std::cout<<"converting to xml \n";
    while (data_stream.fail() == false) {
        ProcessPosVel(data_stream, pos, vel);
        if (data_stream.fail() == false) {
            data_document.AddShape("sphere", .1, pos, QUNIT);
        }

        if(count %1000==0){
         	std::cout<<count<<std::endl;
         }
         count++;

    }
    std::cout<<"write to xml \n";
    stringstream output_file_ss;
    if (argc == 3) {
        output_file_ss << argv[2] << argv[1] << ".xml";
    } else {
        output_file_ss << argv[1] << ".xml";
    }
    data_document.Write(output_file_ss.str());
    return 0;
}
