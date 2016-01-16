#include "converter_general.h"
#include "MitsubaGenerator.h"
#include <chrono_parallel/math/real3.h>
using namespace chrono;

double vehicle_speed, driveshaft_speed, motor_torque, motor_speed, output_torque;
double wheel_torque_0, wheel_torque_1, wheel_torque_2, wheel_torque_3;
real3 wheel_linvel_0, wheel_linvel_1, wheel_linvel_2, wheel_linvel_3;
real3 wheel_angvel_0, wheel_angvel_1, wheel_angvel_2, wheel_angvel_3;
double spring_def_fl, spring_def_fr, spring_def_rl, spring_def_rr;
double shock_len_fl, shock_len_fr, shock_len_rl, shock_len_rr;
double throttle, braking;
std::vector<std::tuple<int, int, std::string> > labels;

void ReadStats(std::string filename) {
    std::ifstream ifile(filename.c_str());

    std::string line;
    std::getline(ifile, line);
    std::replace(line.begin(), line.end(), ',', '\t');
    std::stringstream ss(line);
    ss >> vehicle_speed >> driveshaft_speed >> motor_torque >> motor_speed >> output_torque;
    ss >> wheel_torque_0 >> wheel_torque_1 >> wheel_torque_2 >> wheel_torque_3;

    ss >> wheel_linvel_0.x >> wheel_linvel_0.y >> wheel_linvel_0.z   //
        >> wheel_linvel_1.x >> wheel_linvel_1.y >> wheel_linvel_1.z  //
        >> wheel_linvel_2.x >> wheel_linvel_2.y >> wheel_linvel_2.z  //
        >> wheel_linvel_3.x >> wheel_linvel_3.y >> wheel_linvel_3.z;

    ss >> wheel_angvel_0.x >> wheel_angvel_0.y >> wheel_angvel_0.z   //
        >> wheel_angvel_1.x >> wheel_angvel_1.y >> wheel_angvel_1.z  //
        >> wheel_angvel_2.x >> wheel_angvel_2.y >> wheel_angvel_2.z  //
        >> wheel_angvel_3.x >> wheel_angvel_3.y >> wheel_angvel_3.z;

    ss >> spring_def_fl >> spring_def_fr >> spring_def_rl >> spring_def_rr;
    ss >> shock_len_fl >> shock_len_fr >> shock_len_rl >> shock_len_rr;
    ss >> throttle >> braking;
    ifile.close();

    std::string line_1 = "vehicle speed: " + std::to_string(vehicle_speed) + " [m/s] driveshaft speed: " + std::to_string(driveshaft_speed) + " [rad/s]";
    std::string line_2 = "motor torque: " + std::to_string(motor_torque) + " [Nm] motor speed: " + std::to_string(motor_speed) + " [rad/s] output torque: " +
                         std::to_string(output_torque) + "[Nm]";
    std::string line_3 = "wheel torques: [" + std::to_string(wheel_torque_0) + ", " + std::to_string(wheel_torque_1) + ", " + std::to_string(wheel_torque_2) +
                         ", " + std::to_string(wheel_torque_3) + "] [Nm]";
    std::string line_4 = "throttle: " + std::to_string(throttle) + " brake: " + std::to_string(braking);

    labels.push_back(std::make_tuple(25, 25, line_1));
    labels.push_back(std::make_tuple(25, 50, line_2));
    labels.push_back(std::make_tuple(25, 75, line_3));
    labels.push_back(std::make_tuple(25, 100, line_4));
}

int main(int argc, char* argv[]) {
    if (argc == 1) {
        std::cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << std::endl;
        MitsubaGenerator scene_document("scene.xml");
        scene_document.camera_origin = ChVector<>(0, -10, 0);
        scene_document.camera_target = ChVector<>(0, 0, 0);
        scene_document.camera_up = ChVector<>(0, 0, 1);
        scene_document.scale = 3;
        scene_document.turbidity = 10;
        scene_document.CreateScene();
        // scene_document.AddShape("background", ChVector<>(1), ChVector<>(0, -10,
        // 0), ChQuaternion<>(1, 0, 0, 0));
        scene_document.Write();
        return 0;
    }
    std::stringstream input_file_ss, input_file_vehicle, input_stats_vehicle;
    input_file_ss << "data_" << argv[1] << ".dat";
    input_file_vehicle << "vehicle_" << argv[1] << ".dat";
    input_stats_vehicle << "stats_" << argv[1] << ".dat";

    std::cout << "Opening Files: " << input_file_ss.str() << " " << input_file_vehicle.str() << " " << input_stats_vehicle.str() << "\n";
    std::cout << "Stats\n";
    ReadStats(input_stats_vehicle.str());

    std::ifstream ifile;
    std::cout << "OpenBinary\n";
    OpenBinary(input_file_ss.str(), ifile);

    std::vector<chrono::real3> position;
    std::vector<real3> velocity;
    std::cout << "ReadBinary\n";
    ReadBinary(ifile, position);
    ReadBinary(ifile, velocity);
    std::cout << "CloseBinary\n";
    CloseBinary(ifile);

    std::string data_v;
    std::cout << "ReadCompressed\n";
    ReadCompressed(input_file_vehicle.str(), data_v);
    std::replace(data_v.begin(), data_v.end(), ',', '\t');
    std::stringstream output_file_ss;
    if (argc == 3) {
        output_file_ss << argv[2] << argv[1] << ".xml";
    } else {
        output_file_ss << argv[1] << ".xml";
    }
    MitsubaGenerator data_document(output_file_ss.str());

    std::stringstream vehicle_stream(data_v);

    ChVector<> pos, vel, scale;
    ChQuaternion<> rot;
    int count = 0;
    real max_vel = 0;

    for (int i = 0; i < velocity.size(); i++) {
        vel.x = velocity[i].x;
        vel.y = velocity[i].y;
        vel.z = velocity[i].z;
        max_vel = Max(max_vel, vel.Length());
    }

    for (int i = 0; i < position.size(); i++) {
        pos.x = position[i].x;
        pos.y = position[i].y;
        pos.z = position[i].z;
        vel.x = velocity[i].x;
        vel.y = velocity[i].y;
        vel.z = velocity[i].z;
        double v = vel.Length() / max_vel;

        // data_document.AddShape("sphere", .016, pos, QUNIT);
        data_document.AddCompleteShape("sphere", "diffuse", VelToColor(v), .016 * 2, pos, QUNIT);

        count++;
    }

    // std::cout << data_v << std::endl;

    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("box", scale, pos, rot);
    SkipLine(vehicle_stream, 3);
    //    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    //    data_document.AddShape("box", scale, pos, rot);
    //    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    //    data_document.AddShape("box", scale, pos, rot);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("box", scale, pos, rot);
    SkipLine(vehicle_stream, 1);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("box", scale, pos, rot);
    SkipLine(vehicle_stream, 1);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("box", scale, pos, rot);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("box", scale, pos, rot);

    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    // chrono::ChQuaternion<> q;
    // q.Q_from_AngAxis(CH_C_PI, chrono::ChVector<>(1, 0, 0));
    /// rot = rot * q;
    Vector offset = Vector(0, 0, 0);  // rot.Rotate(Vector(-0.055765, 0, -0.52349));
    data_document.AddShape("chassis", Vector(1, 1, 1), pos + offset, rot);

    Vector camera_pos = pos + offset;
    camera_pos.z = 4;
    camera_pos.y -= 8;
    camera_pos.x += 0;
    data_document.AddSensor(camera_pos, pos + offset, Vector(0, 0, 1), labels);

    SkipLine(vehicle_stream, 5);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("lugged_wheel_R", Vector(1, 1, 1), pos, rot);
    SkipLine(vehicle_stream, 22);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("lugged_wheel_R", Vector(1, -1, 1), pos, rot);
    SkipLine(vehicle_stream, 22);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("lugged_wheel_R", Vector(1, 1, 1), pos, rot);
    SkipLine(vehicle_stream, 22);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("lugged_wheel_R", Vector(1, -1, 1), pos, rot);

    data_document.Write();
    return 0;
}
