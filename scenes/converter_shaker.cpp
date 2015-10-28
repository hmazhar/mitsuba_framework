#include "converter_general.h"
#include "MitsubaGenerator.h"

using namespace std;
using namespace chrono;

int main(int argc, char* argv[]) {
    if (argc == 1) {
        cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << endl;
        MitsubaGenerator scene_document("scene.xml");
        scene_document.camera_origin = ChVector<>(-80, 0, -80);
        scene_document.camera_target = ChVector<>(0, 0, 0);
        scene_document.scale = 3;
        scene_document.turbidity = 10;
        scene_document.CreateScene();
        scene_document.AddShape("background", ChVector<>(1), ChVector<>(0, -10, 0), ChQuaternion<>(1, 0, 0, 0));
        scene_document.Write();
        return 0;
    }
    stringstream input_file_ss;
    input_file_ss << argv[1] << ".dat";

    std::cout << "read compressed... ";
    std::ifstream ifile;

    OpenBinary(input_file_ss.str(), ifile);

    std::vector<half3> position;
    std::vector<half3> velocity;

    ReadBinary(ifile, position);
    ReadBinary(ifile, velocity);
    CloseBinary(ifile);

    std::cout << "reading done\n";

    stringstream output_file_ss;
    if (argc == 3) {
        output_file_ss << argv[2] << argv[1] << ".xml";
    } else {
        output_file_ss << argv[1] << ".xml";
    }

    MitsubaGenerator data_document(output_file_ss.str());

    ChVector<> pos, vel;
    // int count = 0;

    double radius = .1;

    double fstar = .27;
    double Gamma = 3.0;
    double phi = .58;

    double D = radius * 2;
    double L = D * 100;
    double P = 60000;
    double gravity = -9.80665;
    double C_Pi = 3.1415;

    double H = P * C_Pi / 6.0 * (pow(D, 3.0) / pow(L, 2.0)) / phi;
    double frequency = fstar / sqrtf(H / fabs(gravity));
    double amplitude = Gamma * fabs(gravity) * pow(C_Pi, -0.2e1) * pow(frequency, -0.2e1) / 0.4e1;

    double max_vel = ceil(2 * C_Pi * frequency * amplitude) * 2;

    double start = 0;
    double end = max_vel;

    std::cout << "converting to xml \n";
    for (int i = 0; i < position.size(); i++) {
        pos.x = position[i].x;
        pos.y = position[i].y;
        pos.z = position[i].z;
        vel.x = velocity[i].x;
        vel.y = velocity[i].y;
        vel.z = velocity[i].z;
        double v_norm = (vel.Length() - start) / (end - start);

        // data_document.AddShape("sphere", .1, pos, QUNIT);
        data_document.AddCompleteShape("sphere", "diffuse", VelToColor(v_norm), radius, pos, QUNIT);

        //    if (count % 1000 == 0) {
        //      std::cout << count << std::endl;
        //    }
        //  count++;
    }
    std::cout << "write to xml \n";

    data_document.Write();
    return 0;
}
