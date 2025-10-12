#include "lidarModel/lidarModel.hpp"


lidarModel::lidarModel(/* args */)
{
}

lidarModel::~lidarModel()
{
    return;
}

void lidarModel::test(std::shared_ptr<Logger> logger)
{
    logger->logInfo("lidar model test function called");
}

void lidarModel::generatePointCloud(LandmarkList landmarks, std::shared_ptr<Logger> logger)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    double floorOcclusionsDistance[POINTS_PER_ARCH];
    fillOcclusionsArray(floorOcclusionsDistance, landmarks);
    // generateFloorPoints(floorOcclusionsDistance, cloud);

    printConePositions(landmarks, logger);

    for (const auto& lm : landmarks.list) {
        if (lm.type != LandmarkType::BLUE && lm.type != LandmarkType::YELLOW && lm.type != LandmarkType::ORANGE) {
            continue; // skip non-cone landmarks
        }
        uint8_t r, g, b;
        if (lm.type == LandmarkType::YELLOW)
            r = 165, g = 173, b = 3;
        else if (lm.type == LandmarkType::ORANGE)
            r = 255, g = 165, b = 0;
        else if (lm.type == LandmarkType::BLUE)
            r = 0, g = 0, b = 255;

        double c_x = lm.position.x();
        double c_y = lm.position.y();
        double c_z = lm.position.z();
        double distance = std::sqrt(c_x*c_x + c_y*c_y + c_z*c_z);
        double surface = getConeFlattedSurface(lm);
        uint32_t samples = sampleOnCone(surface, distance);
        for (uint32_t i = 0; i < samples; ++i){
            auto [x,y,z] = samplePointOnCone(c_x, c_y, c_z, distance);
            pcl::PointXYZRGB point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.r = r;
            point.g = g;
            point.b = b;
            cloud.push_back(point);
        }
    }


    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;

    pcl::io::savePCDFileASCII("cloud_test.pcd", cloud);

}

void lidarModel::fillOcclusionsArray(double* occlusions, LandmarkList landmarks)
{
    // Inizializza tutte le distanze a un valore molto grande (nessuna ostruzione)
    for (int i = 0; i < POINTS_PER_ARCH; ++i) 
    {
        occlusions[i] = std::numeric_limits<double>::max();
    }

    double min_angle = -45.0 * M_PI / 180.0;
    double max_angle =  45.0 * M_PI / 180.0;

    for (const auto& lm : landmarks.list) 
    {
        if (lm.type != LandmarkType::BLUE && lm.type != LandmarkType::YELLOW && lm.type != LandmarkType::ORANGE) 
        {
            continue; // skip non-cone landmarks
        }
        double x = lm.position.x();
        double y = lm.position.y();
        double z = lm.position.z();
        double distance = std::sqrt(x*x + y*y + z*z);
        // Calcola l'angolo centrale del cono rispetto all'origine
        double theta = std::atan2(y, x);
        // Calcola il raggio (distanza dal centro)
        // Calcola il semiangolo sotteso dalla base del cono
        double alpha = std::asin(RADIUS / distance);

        // Calcola direttamente gli indici degli angoli coperti dal cono senza iterare su tutti
        int start_idx = std::max(0, static_cast<int>(std::ceil((theta - alpha - min_angle) / (max_angle - min_angle) * (POINTS_PER_ARCH - 1))));
        int end_idx = std::min(POINTS_PER_ARCH - 1, static_cast<int>(std::floor((theta + alpha - min_angle) / (max_angle - min_angle) * (POINTS_PER_ARCH - 1))));
        for (int i = start_idx; i <= end_idx; ++i) {
            if (distance < occlusions[i])
                occlusions[i] = distance;
        }
    }
}

void lidarModel::generateFloorPoints(double* occlusions, pcl::PointCloud<pcl::PointXYZRGB>& cloud) 
{
    double min_angle = -45.0 * M_PI / 180.0;
    double max_angle =  45.0 * M_PI / 180.0;
    int num_channel = 28; // 28

    for (int i = 0; i < POINTS_PER_ARCH; ++i) 
    {
        double angle = min_angle + (max_angle - min_angle) * i / (POINTS_PER_ARCH - 1);

        // Compute the max radius for this direction (obstruction or a max range)
        double max_radius = occlusions[i];

        // Instead of radius step, use angle step from the source at height H
        // For each channel, compute the corresponding ground intersection
        for (int ch = 0; ch < num_channel; ++ch) 
        {
            // Vertical angle from the source (from -down to +up)
            // Here, we distribute vertical angles between -25 deg and -0.3 deg (example)
            double min_vert_angle = -25.0 * M_PI / 180.0;
            double max_vert_angle = -0.3 * M_PI / 180.0;
            double vert_angle = min_vert_angle + (max_vert_angle - min_vert_angle) * ch / (num_channel - 1);

            // Avoid division by zero for horizontal rays
            if (std::abs(std::tan(vert_angle)) < 1e-6) continue;

            // Compute ground intersection distance (r) from the source at height H
            double r = LIDAR_Z / -std::tan(vert_angle); // negative tan for downward angles

            if (r <= 0 || r > max_radius) continue;

            pcl::PointXYZRGB point;
            point.x = r * cos(angle);
            point.y = r * sin(angle);
            point.z = 0.0;
            point.r = 128;
            point.g = 128;
            point.b = 128;
            cloud.push_back(point);
        }
    }
}

double lidarModel::getConeFlattedSurface() 
{
    return X_CONE_DIM * Z_CONE_DIM / 2.0;
}

uint32_t lidarModel::sampleOnCone(double surface, double distance) 
{
        // Calcola il numero di campioni in base alla distanza e all'incertezza
    int samples = static_cast<int>(std::ceil(TOTAL_RAY * surface / (2*M_PI * distance * distance * sin(  M_PI / 9 ))));
    return samples;
}

// Implementazione del metodo sampleSurface con z discreta e probabilità decrescente linearmente per valori alti
std::tuple<double, double, double> lidarModel::samplePointOnCone(double pos_x, double pos_y, double pos_z, double distance)  {
    // K è il numero di livelli in cui dividere la z del cono
    int k = 28 - std::atan2(LIDAR_Z, distance) / (24.7 * M_PI / 180.0 / 28.0);

    static std::default_random_engine generator;

    // Probabilità decrescente linearmente per livelli alti di z
    std::vector<double> weights(k);
    double sum = 0.0;
    for (int i = 0; i < k; ++i) {
        weights[i] = static_cast<double>(k - i); // Più basso z_level, più alta la probabilità
        sum += weights[i];
    }
    // Normalizza
    for (int i = 0; i < k; ++i) {
        weights[i] /= sum;
    }

    // Distribuzione discreta pesata
    std::discrete_distribution<int> dist_z(weights.begin(), weights.end());
    int z_level = dist_z(generator);

    double dz = Z_CONE_DIM / k;
    double sample_z = pos_z + z_level * dz;

    double theta = std::acos(RADIUS / distance);
    double alpha = std::atan2(pos_y, pos_x);
    double min = M_PI + alpha - theta;
    double max = M_PI + alpha + theta;

    static std::uniform_real_distribution<double> distribution_phi(0, 1);
    double sample_phi = distribution_phi(generator) * (max - min) + min;

    double radius_at_z = RADIUS * (1 - (sample_z / Z_CONE_DIM));
    double sample_x = pos_x + radius_at_z * cos(sample_phi);
    double sample_y = pos_y + radius_at_z * sin(sample_phi);

    return std::make_tuple(sample_x, sample_y, sample_z);
}


void lidarModel::printConePositions(LandmarkList landmarks, std::shared_ptr<Logger> logger) 
{
    logger->logInfo("Cone positions:");
    std::ofstream outfile("cone_positions.txt", std::ios::app);
    for (const auto& lm : landmarks.list) {
        if (lm.type == LandmarkType::BLUE || lm.type == LandmarkType::YELLOW || lm.type == LandmarkType::ORANGE) {
            std::string type_str;
            if (lm.type == LandmarkType::BLUE)
                type_str = "BLUE";
            else if (lm.type == LandmarkType::YELLOW)
                type_str = "YELLOW";
            else if (lm.type == LandmarkType::ORANGE)
                type_str = "ORANGE";
            else
                type_str = "UNKNOWN";

            std::string info = "Landmark ID: " + std::to_string(lm.id) + " Type: " + type_str
                            + " Position: (" + std::to_string(lm.position.x()) + ", " + std::to_string(lm.position.y())
                            + ", " + std::to_string(lm.position.z()) + ")";
            logger->logInfo(info);
            if (outfile.is_open()) {
                outfile << info << std::endl;
            }
        }
    }
    outfile.close();
}