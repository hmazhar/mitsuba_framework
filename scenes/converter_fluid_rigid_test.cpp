#include "converter_general.h"
#include "MitsubaGenerator.h"
#include <chrono_parallel/math/ChParallelMath.h>
#include <sph_trianglemesh.h>
using namespace chrono;

int main(int argc, char* argv[]) {
    std::ifstream ifile;
    std::cout << "OpenBinary\n";
    for (int file_num = 0; file_num < 588; file_num++) {
        OpenBinary("fluid_" + std::to_string(file_num) + ".dat", ifile);
        std::vector<chrono::real> density;
        std::vector<chrono::real> pressure;
        std::vector<chrono::real3> force;
        std::vector<chrono::real3> position;
        std::vector<real3> velocity;
        std::cout << "ReadBinary\n";
        ReadBinary(ifile, density);
        ReadBinary(ifile, pressure);
        ReadBinary(ifile, force);
        ReadBinary(ifile, position);
        ReadBinary(ifile, velocity);
        std::cout << "CloseBinary " << density.size() << " " << pressure.size() << " " << force.size() << " " << position.size() << " " << velocity.size()
                  << "\n";
        CloseBinary(ifile);

        std::sort(density.begin(), density.end());
        std::sort(pressure.begin(), pressure.end());

        std::vector<chrono::real> force_length(force.size());

        for (int i = 0; i < force.size(); i++) {
            force_length[i] = Length(force[i]);
        }

        std::sort(force_length.begin(), force_length.end());

        real avg_density = 0;
        real avg_pressure = 0;

        for (int i = 0; i < density.size(); i++) {
            avg_density += density[i];
            avg_pressure += pressure[i];
        }
        avg_density = avg_density / density.size();
        avg_pressure = avg_pressure / pressure.size();

        printf("Density, Pressure: [%f %f] \n", avg_density, avg_pressure);
    }

    return 0;
}
