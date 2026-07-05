#include "lidarModel/lidarModel.hpp"


lidarModel::lidarModel(/* args */)
{
    this->current_segment = 0;
    this->num_segments = 1; // default to 1 segment (no distortion)
    this->is_in_dead_time = false;
}

lidarModel::~lidarModel()
{
    return;
}

void lidarModel::test(std::shared_ptr<Logger> logger)
{
    logger->logInfo("lidar model test function called");
}

pcl::PointCloud<pcl::PointXYZRGB> lidarModel::generatePointCloud(LandmarkList landmarks, std::shared_ptr<Logger> logger)
{
    // Backward compatibility: generate full cloud in one go
    pcl::PointCloud<pcl::PointXYZRGB> out_cloud;
    this->current_segment = 0;
    uint16_t original_segments = this->num_segments;
    this->num_segments = 1;
    generateSegment(landmarks, out_cloud, logger);
    this->num_segments = original_segments;
    return out_cloud;
}

bool lidarModel::generateSegment(LandmarkList landmarks, pcl::PointCloud<pcl::PointXYZRGB>& out_cloud, std::shared_ptr<Logger> logger)
{
    if (this->current_segment == 0) {
        this->accumulated_cloud.clear();
    }

    int idx_per_segment = this->points_per_arch / this->num_segments;
    int start_idx = this->current_segment * idx_per_segment;
    int end_idx = (this->current_segment == this->num_segments - 1) ? (this->points_per_arch - 1) : (start_idx + idx_per_segment - 1);

    double phi_start = this->min_angle_horizontal + (this->max_angle_horizontal - this->min_angle_horizontal) * start_idx / (this->points_per_arch - 1);
    double phi_end = this->min_angle_horizontal + (this->max_angle_horizontal - this->min_angle_horizontal) * (end_idx + 1) / (this->points_per_arch - 1);

    std::vector<double> floorOcclusionsDistance(this->points_per_arch);
    fillOcclusionsArray(floorOcclusionsDistance.data(), landmarks, start_idx, end_idx);
    generateFloorPoints(floorOcclusionsDistance.data(), this->accumulated_cloud, start_idx, end_idx);

    for (const auto& lm : landmarks.list) {
        if (lm.type != LandmarkType::BLUE && lm.type != LandmarkType::YELLOW && lm.type != LandmarkType::ORANGE) {
            continue; // skip non-cone landmarks
        }

        if (!isConeInSegment(lm, phi_start, phi_end)) {
            continue;
        }

        uint8_t r, g, b;
        if (lm.type == LandmarkType::BLUE) {
            r = 50; g = 150; b = 255; // Light/Bright Blue, visible on dark background
        } else if (lm.type == LandmarkType::YELLOW) {
            r = 255; g = 255; b = 0; // Yellow
        } else if (lm.type == LandmarkType::ORANGE) {
            r = 255; g = 150; b = 0; // Bright Orange
        } else {
            r = 255; g = 255; b = 255;
        }

        double c_x = lm.position.x();
        double c_y = lm.position.y();
        double c_z = lm.position.z();
        double distance = std::sqrt(c_x*c_x + c_y*c_y + c_z*c_z);
        double surface = getConeFlattedSurface();
        uint32_t samples = sampleOnCone(surface, distance);
        for (uint32_t i = 0; i < samples; ++i){
            auto [x,y,z] = samplePointOnCone(c_x, c_y, c_z, distance);

            // Point-wise filtering
            double phi_point = std::atan2(y + this->lidar_y, x + this->lidar_x); // back to sensor frame for filtering
            if (phi_point >= phi_start && phi_point < phi_end) {
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.r = r;
                point.g = g;
                point.b = b;
                this->accumulated_cloud.push_back(point);
            }
        }
    }

    this->current_segment++;

    if (this->current_segment >= this->num_segments) {
        this->current_segment = 0;
        applyNoise(this->accumulated_cloud);

        this->accumulated_cloud.width = this->accumulated_cloud.points.size();
        this->accumulated_cloud.height = 1;
        this->accumulated_cloud.is_dense = false;
        out_cloud = this->accumulated_cloud;
        return true;
    }

    return false;
}

bool lidarModel::isConeInSegment(const Landmark& lm, double phi_start, double phi_end)
{
    double dist_to_center = std::sqrt(lm.position.x()*lm.position.x() + lm.position.y()*lm.position.y());
    double theta_center = std::atan2(lm.position.y(), lm.position.x());
    double alpha_width = std::asin(RADIUS / dist_to_center);

    return std::max(theta_center - alpha_width, phi_start) <= std::min(theta_center + alpha_width, phi_end);
}

void lidarModel::applyNoise(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    static std::default_random_engine generator(std::random_device{}());
    for (auto& point : cloud.points) {
        double D = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (D > 0) {
            double std_dev = D * std::tan(this->angular_uncertainty);
            std::normal_distribution<double> dist(0.0, std_dev);
            point.x += dist(generator);
            point.y += dist(generator);
            point.z += dist(generator);
        }
    }
}


void lidarModel::fillOcclusionsArray(double* occlusions, LandmarkList landmarks, int start_idx, int end_idx)
{
    // Inizializza solo la porzione del segmento
    for (int i = start_idx; i <= end_idx; ++i)
    {
        occlusions[i] = std::numeric_limits<double>::max();
    }

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
        int lm_start_idx = std::max(start_idx, static_cast<int>(std::ceil((theta - alpha - this->min_angle_horizontal) / (this->max_angle_horizontal - this->min_angle_horizontal) * (this->points_per_arch - 1))));
        int lm_end_idx = std::min(end_idx, static_cast<int>(std::floor((theta + alpha - this->min_angle_horizontal) / (this->max_angle_horizontal - this->min_angle_horizontal) * (this->points_per_arch - 1))));
        for (int i = lm_start_idx; i <= lm_end_idx; ++i) {
            if (distance < occlusions[i])
                occlusions[i] = distance - RADIUS;
        }
    }
}

void lidarModel::generateFloorPoints(double* occlusions, pcl::PointCloud<pcl::PointXYZRGB>& cloud, int start_idx, int end_idx)
{
    for (int i = start_idx; i <= end_idx; ++i)
    {
        double angle = this->min_angle_horizontal + (this->max_angle_horizontal - this->min_angle_horizontal) * i / (this->points_per_arch - 1);

        // Compute the max radius for this direction (obstruction or a max range)
        double max_radius = occlusions[i];

        // Instead of radius step, use angle step from the source at height H
        // For each channel, compute the corresponding ground intersection
        for (int ch = 0; ch < this->num_channel; ++ch)
        {
            // Vertical angle from the source (from -down to +up)
            // Here, we distribute vertical angles between -25 deg and -0.3 deg (example)
            double min_vert_angle = -25.0 * M_PI / 180.0;
            double max_vert_angle = -0.3 * M_PI / 180.0;
            double vert_angle = min_vert_angle + (max_vert_angle - min_vert_angle) * ch / (this->num_channel - 1);

            // Avoid division by zero for horizontal rays
            if (std::abs(std::tan(vert_angle)) < 1e-6) continue;

            // Compute ground intersection distance (r) from the source at height H
            double r = this->lidar_z / -std::tan(vert_angle); // negative tan for downward angles

            if (r <= 0 || r > max_radius) continue;

            pcl::PointXYZRGB point;
            point.x = r * cos(angle) - this->lidar_x;
            point.y = r * sin(angle) - this->lidar_y;
            point.z = -this->lidar_z; 
            
            // Floor point color
            point.r = 100;
            point.g = 100;
            point.b = 100;
            
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
    int samples = static_cast<int>(std::ceil(this->total_ray * surface / (2*M_PI * distance * distance * sin(  M_PI / 9 ))));
    return samples;
}

int lidarModel::countChannelsFast(double D)
{
    double min_vert_angle = 25.0 * M_PI / 180.0;
    double max_vert_angle = 0.3  * M_PI / 180.0;

    double delta = (max_vert_angle - min_vert_angle) / (this->num_channel - 1);

    double theta_thr = atan(this->lidar_z / D);

    int k_min = static_cast<int>(std::ceil(
        (theta_thr - min_vert_angle) / delta
    ));

    if (k_min < 0) return this->num_channel;
    if (k_min >= this->num_channel) return 0;
    return this->num_channel - k_min;
}

// Implementazione del metodo sampleSurface con z discreta e probabilità decrescente linearmente per valori alti
std::tuple<double, double, double> lidarModel::samplePointOnCone(double pos_x, double pos_y, double pos_z, double distance)  {
    // K è il numero di livelli in cui dividere la z del cono
    int k = countChannelsFast(distance);

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

    return std::make_tuple(sample_x - this->lidar_x, sample_y - this->lidar_y, sample_z - this->lidar_z); // Position relative to LiDAR
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

void lidarModel::readConfig(ConfigElement& config)
{
    config.getElement<uint32_t>(&this->total_ray, "total_ray");
    config.getElement<uint16_t>(&this->points_per_arch, "points_per_arch");
    config.getElement<uint16_t>(&this->num_channel, "num_channel");
    config.getElement<double>(&this->min_angle_horizontal, "min_angle_horizontal");
    config.getElement<double>(&this->max_angle_horizontal, "max_angle_horizontal");
    config.getElement<double>(&this->lidar_x, "lidar_x");
    config.getElement<double>(&this->lidar_y, "lidar_y");
    config.getElement<double>(&this->lidar_z, "lidar_z");
    try {
        config.getElement<double>(&this->angular_uncertainty, "angular_uncertainty");
    } catch (const std::exception& e) {
        this->angular_uncertainty = 0.005; // default fallback
    }
    try {
        uint32_t segments;
        config.getElement<uint32_t>(&segments, "num_segments");
        this->num_segments = static_cast<uint16_t>(segments);
    } catch (const std::exception& e) {
        this->num_segments = 1; // default fallback
    }
    try {
        config.getElement<double>(&this->rate, "rate");
    } catch (const std::exception& e) {
        this->rate = 10.0; // default fallback
    }
    try {
        config.getElement<std::string>(&this->perception_sensor_name, "perception_sensor_name");
    } catch (const std::exception& e) {
        this->perception_sensor_name = "livox_front"; // default fallback
    }
}

bool lidarModel::RunTick(double simTime, LandmarkList& trackAsLMList, Eigen::Vector3d t, Eigen::Vector3d rEulerAngles, pcl::PointCloud<pcl::PointXYZRGB>& out_cloud, std::shared_ptr<Logger> logger)
{
    if (!perceptionSensor) return false;

    double fov = std::abs(this->max_angle_horizontal - this->min_angle_horizontal);
    double t_total = 1.0 / this->rate;
    double t_active = t_total * (fov / (2.0 * M_PI));
    double dt_segment = t_active / this->num_segments;
    double t_dead = t_total - t_active;

    // Gestione dell'eventuale tempo morto a fine giro
    if (this->is_in_dead_time) {
        if (simTime < (this->lastLidarSegmentTime + t_dead)) {
            return false; // Il lidar sta girando a vuoto, aspettiamo
        }
        
        // Fine della zona morta: aggiorniamo il tempo e pubblichiamo la cloud
        this->lastLidarSegmentTime += t_dead;
        this->is_in_dead_time = false;
        out_cloud = this->accumulated_cloud;
        return true;
    }

    // Acquisizione dei segmenti attivi
    if (simTime < (this->lastLidarSegmentTime + dt_segment)) {
        return false; // Non è ancora il momento di acquisire il prossimo segmento
    }

    // Eseguiamo l'acquisizione del segmento
    this->lastLidarSegmentTime += dt_segment;
    LandmarkList sensorLmsSegment = perceptionSensor->process(trackAsLMList, t, rEulerAngles, simTime);
    bool finished_active = this->generateSegment(sensorLmsSegment, out_cloud, logger);

    // Se non abbiamo finito tutti i segmenti del FOV, aspettiamo i prossimi
    if (!finished_active) {
        return false;
    }

    // Abbiamo completato il FOV. Dobbiamo simulare latenza?
    if (t_dead > 1e-6) {
        this->is_in_dead_time = true;
        return false; // Non pubblichiamo ancora, entriamo nello stato di latenza
    }
    
    // Niente latenza (es. lidar a 360°), pubblichiamo subito
    return true;
}

