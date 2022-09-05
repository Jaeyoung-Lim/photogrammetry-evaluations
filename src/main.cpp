#include <iostream>
#include <getopt.h>

#include <Eigen/Dense>

int main(int argc, char *argv[]) {
    std::cout << "Hello, world!\n";
    ///TODO: Add configuration loader
    // std::string groundtruth_mesh_path, reconstructed_mesh_path;
    // for(;;) {
    //     switch(getopt(argc, argv, "g:r:h")) {
    //         case 'g':
    //             groundtruth_mesh_path=std::string(optarg);
    //         case 'r':
    //             reconstructed_mesh_path=std::string(optarg);
    //         case '?':
    //         case 'h':
    //         default :
    //             printf("Help/Usage Example\n");
    //         break;
    //     }

    //     break;
    // }
    std::cout << "Starting evaluation of photogrammetry results" << std::endl;

    ///TODO: Load groundtruth mesh
    // std::cout << " - Groundtruth Mesh path: " << groundtruth_mesh_path << std::endl;
    ///TODO: Load reconstructed mesh
    // std::cout << " - Reconstructed Mesh path: " << groundtruth_mesh_path << std::endl;
    ///TODO: Compute accuracy
    ///TODO: Compute completeness

    auto airsim_camera_position = Eigen::Vector4d(224.35, -331.356, -24.3878, 1.0);
    auto colmap_camera_position = Eigen::Vector4d(-0.98, 1.56, -0.43, 1.0);
    auto player_start = Eigen::Vector3d(374.47859375, -723.12984375, -286.77371094);

    Eigen::Matrix4d transform;
    transform << 0.84240145816622813, -0.0013301086479464012, 0.53884878592272734, 172.80954006060944,
        0.53885042755797596, 0.0020793997874840107, -0.8423988917473193, -270.15847271723669,
        -3.1225022567582522e-17, 0.9999969534491141, 0.0024684190267316992, -104.60412502665753,
        0, 0, 0,1;

    std::cout << "Airsim camera position: " << airsim_camera_position.transpose() << std::endl;
    std::cout << "Colmap camera position: " << colmap_camera_position.transpose() << std::endl;

    std::cout << "Player start: " << player_start.transpose() << std::endl;

    Eigen::Vector4d transformed_camera_position = transform * colmap_camera_position;
    std::cout << "  Transformed camera position: " << transformed_camera_position.transpose() << std::endl;
    // std::cout << "  Error: " << transformed_camera_position - 
}
