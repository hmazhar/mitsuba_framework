#include "converter_general.h"
#include "MitsubaGenerator.h"
#include <chrono_parallel/math/ChParallelMath.h>
#include <sph_trianglemesh.h>
using namespace chrono;

double vehicle_speed, driveshaft_speed, motor_torque, motor_speed, output_torque;
double sprocket_torque_left, sprocket_torque_right, wheel_torque_2, wheel_torque_3;
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

    ChVector<> chassis_pos;
    real total_force = 0, ftotal_force = 0;
    real total_torque = 0, ftotal_torque = 0;
    ss >> chassis_pos.x >> chassis_pos.y >> chassis_pos.z;
    ss >> vehicle_speed >> driveshaft_speed >> motor_torque >> motor_speed >> output_torque;

    ss >> throttle >> braking;

    ss >> total_force >> total_torque;
    ss >> ftotal_force >> ftotal_torque;
    ss >> sprocket_torque_left >> sprocket_torque_right;

    ifile.close();

    std::string line_1 = "vehicle speed: " + std::to_string(vehicle_speed) + " [m/s] driveshaft speed: " + std::to_string(driveshaft_speed) +
                         " [rad/s] motor speed: " + std::to_string(motor_speed);
    //    std::string line_2 = "motor torque: " + std::to_string(motor_torque) + " [Nm] motor speed: " + std::to_string(motor_speed) + " [rad/s] output torque:
    //    " + std::to_string(output_torque) + "[Nm]";
    std::string line_3 = "sprocket torques: [" + std::to_string(sprocket_torque_left) + ", " + std::to_string(sprocket_torque_right) + ", " + std::to_string(wheel_torque_2) +
                         ", " + std::to_string(wheel_torque_3) + "] [Nm]";
    std::string line_4 = "throttle: " + std::to_string(throttle) + " brake: " + std::to_string(braking);

    std::string line_5 = "Force on Vehicle [Total, Fluid]: [" + std::to_string(total_force) + ", " + std::to_string(ftotal_force) + "] [N]";

    labels.push_back(std::make_tuple(25, 25, line_1));
    // labels.push_back(std::make_tuple(25, 50, line_2));
    labels.push_back(std::make_tuple(25, 50, line_3));
    labels.push_back(std::make_tuple(25, 75, line_4));
    labels.push_back(std::make_tuple(25, 100, line_5));
}

int main(int argc, char* argv[]) {
    if (argc == 1) {
        std::cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << std::endl;
		std::cout << "[Frame number] [folder] [color_mode] [camera mode]" << std::endl;
        MitsubaGenerator scene_document("scene.xml");
        std::vector<xml_option> integrator_options = {xml_option("boolean", "hideEmitters", "true"), xml_option("integer", "maxDepth", "20"), xml_option("integer", "rrDepth", "10")};
        std::vector<xml_option> emitter_options = {xml_option("string", "filename", "interior_hdri_2_20150408_1285285587.jpg"),  xml_option("float", "scale", "4.000000")};
        scene_document.AddInclude("geometry.xml");
        scene_document.AddIntegrator("volpath", integrator_options);
        scene_document.AddEmitter("envmap", emitter_options, ChVector<>(1, 1, 1), ChVector<>(0, 0, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));
        scene_document.AddInclude("$frame.xml");
        scene_document.Write();
        return 0;
    }
    std::stringstream input_file_fluid, input_file_vehicle, input_stats_vehicle;

	input_file_fluid << "data_" << argv[1] << ".dat";
    input_file_vehicle << "vehicle_" << argv[1] << ".dat";
    input_stats_vehicle << "stats_" << argv[1] << ".dat";

    std::cout << "Opening Files: " << input_file_fluid.str() << " " << input_file_vehicle.str() << " " << input_stats_vehicle.str() << "\n";
    std::cout << "Read Stats\n";
    ReadStats(input_stats_vehicle.str());

    std::ifstream fluid_file;
    std::cout << "OpenBinary\n";
    OpenBinary(input_file_fluid.str(), fluid_file);

    std::vector<chrono::real3> position;
    std::vector<real3> velocity;

    std::cout << "ReadBinary\n";
    ReadBinary(fluid_file, position);
    ReadBinary(fluid_file, velocity);
    std::cout << "CloseBinary\n";
	std::cout << "Read "<<position.size()<<" fluid particles\n";
    CloseBinary(fluid_file);

    std::string data_v;
    std::cout << "ReadCompressed\n";
    ReadCompressed(input_file_vehicle.str(), data_v);
    std::replace(data_v.begin(), data_v.end(), ',', ' ');
	std::replace(data_v.begin(), data_v.end(), char(0), ' ');
	std::cout << data_v << std::endl;

    std::stringstream output_file_ss;
    std::stringstream output_mesh_ss;
    if (argc >= 3) {
        output_file_ss << argv[2] << argv[1] << ".xml";
        output_mesh_ss << argv[2] << argv[1] << ".obj";
        std::cout << "Writing Files: " << output_file_ss.str() << " " << output_mesh_ss.str() << "\n";

    } else {
        output_file_ss << argv[1] << ".xml";
        output_mesh_ss << argv[1] << ".obj";
    }

    int color_mode = 1;
    int follow_camera = 1;

    if (argc >= 5) {
        color_mode = atoi(argv[3]);
        follow_camera = atoi(argv[4]);
    }

    MitsubaGenerator data_document(output_file_ss.str());

    std::stringstream vehicle_stream(data_v);

    ChVector<> pos, vel, scale;
	ChQuaternion<> rot;

    std::vector<double> lengths(velocity.size());

    int count = 0;
    real max_vel = 0;
    real avg_vel = 0;
    for (int i = 0; i < velocity.size(); i++) {
        vel.x = velocity[i].x;
        vel.y = velocity[i].y;
        vel.z = velocity[i].z;

        lengths[i] = vel.Length();
        avg_vel += vel.Length();
    }
    avg_vel /= velocity.size();
    std::sort(lengths.begin(), lengths.end());
    max_vel = lengths[velocity.size() - velocity.size() * .1];

    real variance = 0;
    for (int i = 0; i < velocity.size(); i++) {
        variance += (lengths[i] - avg_vel) * (lengths[i] - avg_vel);
    }
    variance /= velocity.size();

    real std_dev = sqrt(variance);
    printf("mean: %f, stddev: %f, max: %f\n", avg_vel, std_dev, max_vel);

    real kernel_radius = .016;
    if (color_mode > 0 || argc >= 6) {
		printf("Adding fluid particles as spheres\n");
        for (int i = 0; i < position.size(); i++) {
            pos.x = position[i].x;
            pos.y = position[i].y;
            pos.z = position[i].z;

            if (color_mode == 1) {
                vel.x = velocity[i].x;
                vel.y = velocity[i].y;
                vel.z = velocity[i].z;

                double v = vel.Length() / max_vel;
                data_document.AddCompleteShape("sphere", "diffuse", VelToColor(v), kernel_radius, pos, QUNIT);
            } else {
                data_document.AddShape("sphere", kernel_radius, pos, QUNIT);
            }
            count++;
        }
    }
    // Always output if fluid sim (because geometry file has it...)
    if (argc < 6) {
        MarchingCubesToMesh(position, kernel_radius, output_mesh_ss.str(), real3(-200, -200, -3), real3(200, 200, 9), 0.000001);
        data_document.AddShape("fluid", ChVector<>(1), ChVector<>(0), QUNIT);
    }
	//platform
	ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
	//data_document.AddShape("box", scale, pos, rot);

	//for (int trough = 0; trough < 14; trough++) {

	//	int type = ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
	//	if (trough ==3 || trough == 4 || trough ==5 || trough == 8 || trough == 11) {
	//		continue;
	//	}

	//	if (vehicle_stream.fail() == false) {
	//		switch (type) {
	//		case chrono::collision::SPHERE:
	//			printf("sphere %d %d %d %d \n", trough, scale.x, scale.y, scale.z);
	//			data_document.AddShape("sphere", scale, pos, rot);
	//			break;
	//		case chrono::collision::ELLIPSOID:
	//			data_document.AddShape("ellipsoid", scale, pos, rot);
	//			break;
	//		case chrono::collision::BOX:
	//			data_document.AddShape("box", scale, pos, rot);
	//			break;
	//		case chrono::collision::CYLINDER:
	//			/*if (scale.x == .223) {
	//				scale.y = .125;
	//			}
	//			data_document.AddShape("cylinder", scale, pos, rot);*/
	//			break;
	//		case chrono::collision::CONE:
	//			data_document.AddShape("cone", scale, pos, rot);
	//			break;
	//		case (-1) :
	//			break;
	//		}
	//	}
	//}

    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);

    /// rot = rot * q;
    Vector offset = Vector(0, 0, 0);  // rot.Rotate(Vector(-0.055765, 0, -0.52349));
    data_document.AddShape("chassis", Vector(1, 1, 1), pos + offset, rot);
    std::vector<xml_option> sampler_options = {xml_option("integer", "sampleCount", "256"), xml_option("integer", "scramble", argv[1])};
    if (follow_camera == 1) {
        Vector camera_pos = Vector(0,0,0);
       // camera_pos.z = 3;
       // camera_pos.y -= 8;
       // camera_pos.x += 6;

		Vector camera_vec = pos;
		camera_vec.Normalize();
        data_document.AddShape("background", ChVector<>(1, 1, 1), ChVector<>(0,0, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));
        data_document.AddSensor(camera_pos + camera_vec * 22 + Vector(0, 0, 3), pos + offset, Vector(0, 0, 1), labels, "sobol", sampler_options);
		std::cout << "CAM1 \n";


    } else  if (follow_camera == 2) {
		Vector camera_pos = Vector(0, 0, 0);
		// camera_pos.z = 3;
		// camera_pos.y -= 8;
		// camera_pos.x += 6;

		Vector camera_vec = pos;
		camera_vec.Normalize();
		data_document.AddShape("background", ChVector<>(1, 1, 1), ChVector<>(0, -125, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));
		data_document.AddSensor(pos + Vector(0, -8, 3), pos + offset, Vector(0, 0, 1), labels, "sobol", sampler_options);
		std::cout << "CAM1 \n";


	}
	else {
        Vector camera_pos = ChVector<>(0, -9, 3);
        Vector camera_target = ChVector<>(0, -6.4, 2.84);
        //data_document.AddShape("background", ChVector<>(20, 20, 5), ChVector<>(0, 2.1366, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));
        data_document.AddSensor(camera_pos, camera_target, Vector(0, 0, 1), labels, "sobol", sampler_options);
    }

	SkipLine(vehicle_stream, 32);
	SkipLine(vehicle_stream, 5);
	ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
	data_document.AddShape("wheel_R", Vector(1, 1, 1), pos, rot);
	SkipLine(vehicle_stream, 9);
	ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
	data_document.AddShape("wheel_R", Vector(1, -1, 1), pos, rot);
	SkipLine(vehicle_stream, 9);
	ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
	data_document.AddShape("wheel_R", Vector(1, 1, 1), pos, rot);
	SkipLine(vehicle_stream, 9);
	ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
	data_document.AddShape("wheel_R", Vector(1, -1, 1), pos, rot);
	
	//
	//int cyl_c = 0;
 //       while (vehicle_stream.fail() == false) {
 //           int type = ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
 //           if (vehicle_stream.fail() == false) {
 //               switch (type) {
 //                   case chrono::collision::SPHERE:
 //                       data_document.AddShape("sphere", scale, pos, rot);
 //                       break;
 //                   case chrono::collision::ELLIPSOID:
 //                       data_document.AddShape("ellipsoid", scale, pos, rot);
 //                       break;
 //                   case chrono::collision::BOX:
 //                      // data_document.AddShape("box", scale, pos, rot);
 //                       break;
 //                   case chrono::collision::CYLINDER:
	//					std::cout << "cylinder\n";
	//					if (scale.y==0.254) {
	//						if (cyl_c == 2 || cyl_c == 3)
	//						{
	//							data_document.AddShape("wheel_R", scale, pos, rot);
	//						}
	//						else if (cyl_c == 4 || cyl_c == 5)
	//						{
	//							data_document.AddShape("wheel_L", scale, pos, rot);
	//						}
	//						cyl_c++;
	//					}
	//					
	//					else {
	//						//data_document.AddShape("cylinder", scale, pos, rot);
	//					}
	//						
 //                       break;
 //                   case chrono::collision::CONE:
 //                       data_document.AddShape("cone", scale, pos, rot);
 //                       break;
 //                   case (-1):
 //                       break;
 //               }
 //           }
 //       }
    data_document.Write();
    return 0;
}
