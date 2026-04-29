#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>
#include "json.hpp"
using json = nlohmann::json;

// Ammunition types and their properties
constexpr char conf_file[] = "config.json";
constexpr char ammo_file[] = "ammo.json";
constexpr char targets_file[] = "targets.json";
constexpr char out_file[] = "simulation.json";

constexpr float g = 9.81f;
constexpr int MAX_STEPS = 10000;

// Ammunition types and their properties
constexpr int TARGET_COUNT = 5;
constexpr int TARGET_STEPS = 60;

// Drone states
enum DroneState
{
    STOPPED,
    ACCELERATING,
    DECELERATING,
    TURNING,
    MOVING,
};

struct Coord
{
    float x;
    float y;
};

struct DroneConfig
{
    Coord startPos;      // Drone coordinates (x, y (m))
    float altitude;      // altitude (m)
    float initialDir;    // Drone attack speed (m/s)
    float attackSpeed;   // Drone attack speed (m/s)
    float accelPath;     // Drone acceleration distance before attack (m)
    char ammoName[32];   // Ammunition name
    float arrayTimeStep; // Time step for Target coordinates array (sec)
    float simTimeStep;   // Time step for Simulation (sec)
    float hitRadius;     // Angular spped of rotation (rad/sec)
    float angularSpeed;  // Angular spped of rotation (rad/sec)
    float turnThreshold; // Threshold angle for stopping
};

void readIn(DroneConfig &config, const char *conf_file)
{
    std::ifstream input(conf_file);
    if (!input.is_open())
    {
        std::cerr << "Failed to open  " << conf_file << std::endl;
        return;
    }
    json j;
    input >> j;
    input.close();

    config.startPos.x = j["drone"]["position"]["x"];
    config.startPos.y = j["drone"]["position"]["y"];
    config.altitude = j["drone"]["altitude"];
    config.initialDir = j["drone"]["initialDirection"];
    config.attackSpeed = j["drone"]["attackSpeed"];
    config.accelPath = j["drone"]["accelerationPath"];
    config.arrayTimeStep = j["targetArrayTimeStep"];
    config.simTimeStep = j["simulation"]["timeStep"];
    config.hitRadius = j["simulation"]["hitRadius"];
    config.angularSpeed = j["drone"]["angularSpeed"];
    config.turnThreshold = j["drone"]["turnThreshold"];
    const char *tmp = j["ammo"].get<std::string>().c_str();
    std::strncpy(config.ammoName, tmp, 31);

    input.close();
}

struct AmmoParams
{
    char name[32];
    float mass; // mass (kg)
    float drag; // drag coefficient
    float lift; // lift coeficient
};

bool getAmmunitionProperties(AmmoParams &ammo, const char *ammo_file)
{
    std::ifstream input(ammo_file);
    if (!input.is_open())
    {
        std::cerr << "Failed to open  " << ammo_file << std::endl;
        return false;
    }
    json ja;
    input >> ja;
    input.close();

    int ammoCount = ja.size();
    for (int i = 0; i < ammoCount; i++)
    {
        if (std::strcmp(ja[i]["name"].get<std::string>().c_str(), ammo.name) == 0)
        {
            ammo.mass = ja[i]["mass"];
            ammo.drag = ja[i]["drag"];
            ammo.lift = ja[i]["lift"];
        }
    }

    return true;
}

void readTargets(float targetXInTime[TARGET_COUNT][TARGET_STEPS], float targetYInTime[TARGET_COUNT][TARGET_STEPS], const char *targets_file)
{
    std::ifstream input(targets_file);
    if (!input.is_open())
    {
        std::cerr << "Failed to open  " << targets_file << std::endl;
        return;
    }

    json jt;
    input >> jt;
    input.close();

    int tgtCount = jt["targetCount"];
    int timeSteps = jt["timeSteps"];

    Coord **targets = new Coord *[tgtCount];
    for (int i = 0; i < tgtCount; i++)
    {
        targets[i] = new Coord[timeSteps];
        for (int j = 0; j < timeSteps; j++)
        {
            targets[i][j].x = jt["targets"][i]["positions"][j]["x"];
            targets[i][j].y = jt["targets"][i]["positions"][j]["y"];
        }
    }
    for (int i = 0; i < tgtCount; i++)
        delete[] targets[i];
    delete[] targets;
}

double getT(float m, float d, float l, float attackSpeed, float zd)
{
    double a = d * g * m - 2 * pow(d, 2) * l * attackSpeed;
    double b = -3 * g * pow(m, 2) + 3 * d * l * m * attackSpeed;
    double c = 6 * pow(m, 2) * zd;

    double p = -1 * pow(b, 2) / (3 * pow(a, 2));
    double q = 2 * pow(b, 3) / (27 * pow(a, 3)) + c / a;

    double phi = std::acos(3 * q / (2 * p) * std::sqrt(-3 / p));

    double t = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * M_PI) / 3) - b / (3 * a);
    return t;
}

double getH(double t, float attackSpeed, float d, float m, float l)
{
    double h = attackSpeed * t - pow(t, 2) * d * attackSpeed / (2 * m) + pow(t, 3) * (6 * d * g * l * m - 6 * pow(d, 2) * (pow(l, 2) - 1) * attackSpeed) / (36 * pow(m, 2)) +
               pow(t, 4) * (-6 * pow(d, 2) * g * l * (1 + pow(l, 2) + pow(l, 4)) * m + 3 * pow(d, 3) * pow(l, 2) * (1 + pow(l, 2)) * attackSpeed + 6 * pow(d, 3) * pow(l, 4) * (1 + pow(l, 2)) * attackSpeed) / (pow((36 * (1 + pow(l, 2))), 2) * pow(m, 3)) + pow(t, 5) * (3 * pow(d, 3) * g * pow(l, 3) * m - 3 * pow(d, 4) * pow(l, 2) * (1 + pow(l, 2)) * attackSpeed) / (36 * (1 + pow(l, 2)) * pow(m, 4));
    return h;
}

float getDronPosition(float targetXYZ, float simTimeStep, float t)
{
    float dp = targetXYZ * (t + simTimeStep) - targetXYZ * (t);

    return dp;
}

float getTargetSpeed(float targetXYZ, float arrayTimeStep, double t)
{
    float dxyz = targetXYZ * (t + arrayTimeStep) - targetXYZ * (t);
    float targetVxyz = dxyz / arrayTimeStep;

    return targetVxyz;
}

float getPredictedPosition(float targetXYZ, float arrayTimeStep, float t, float totalTime)
{
    float targetVxyz = getTargetSpeed(targetXYZ, arrayTimeStep, t);

    float predictedXYZ = targetXYZ * (t) + targetVxyz * totalTime;

    return predictedXYZ;
}

double getDistanceToTarget(float xd, float yd, float targetX, float targetY)
{
    return sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));
}

float getRatio(float distance_to_target, double h)
{
    return (distance_to_target - h) / distance_to_target;
}

float getTimeToStop(DroneState phase, DroneConfig config, float acceleration)
{
    switch (phase)
    {
    case STOPPED:
        return 0;

    case ACCELERATING:
        return 0;

    case DECELERATING:
        return 0;

    case TURNING:
        return 0;

    case MOVING:
        return config.attackSpeed / acceleration;

    default:
        return 0;
    }
}

int main()
{
    // Read input parameters from file
    DroneConfig drone;

    readIn(drone, conf_file);

    std::cout << "Input Data:" << std::endl;
    std::cout << "Drone coordinates: (" << drone.startPos.x << ", " << drone.startPos.y << ", " << drone.altitude << ")" << std::endl;
    std::cout << "Initial drone direction: " << drone.initialDir << " degrees" << std::endl;
    std::cout << "Attack speed: " << drone.attackSpeed << " m/s" << std::endl;
    std::cout << "Acceleration  path: " << drone.accelPath << " m" << std::endl;
    std::cout << "Ammunition: " << drone.ammoName << std::endl;
    std::cout << "Time step for Target coordinates array: " << drone.arrayTimeStep << " sec" << std::endl;
    std::cout << "Time step for Simulation: " << drone.simTimeStep << " sec" << std::endl;
    std::cout << "Kill zone radius: " << drone.hitRadius << " m" << std::endl;
    std::cout << "Angular speed of rotation: " << drone.angularSpeed << " rad/sec" << std::endl;
    std::cout << "Threshold angle for stopping: " << drone.turnThreshold << " degrees" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    AmmoParams ammo;
    std::strcpy(ammo.name, drone.ammoName);

    // Get ammunition properties
    if (!getAmmunitionProperties(ammo, ammo_file))
    {
        return 1;
    }

    std::cout << "Ammunition properties: " << std::endl;
    std::cout << "Name: " << ammo.name << std::endl;
    std::cout << "Mass: " << ammo.mass << " kg" << std::endl;
    std::cout << "Drag coefficient: " << ammo.drag << std::endl;
    std::cout << "Lift coefficient: " << ammo.lift << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    // Drone state
    DroneState phase = STOPPED;

    // Read target coordinates from file
    // Target coordinates
    float targetXInTime[TARGET_COUNT][TARGET_STEPS];
    float targetYInTime[TARGET_COUNT][TARGET_STEPS];

    readTargets(targetXInTime, targetYInTime, targets_file);

    // The fall time "t" is determined from the quadratic equation of motion with drag and lift
    // We will use Cardano formula to solve the cubic equation for the time of flight considering drag and lift
    double t{getT(ammo.mass, ammo.drag, ammo.lift, drone.attackSpeed, drone.altitude)};
    std::cout << "Flight time: " << t << " seconds" << std::endl;

    // Horisontal flight distance
    double h{getH(t, drone.attackSpeed, ammo.mass, ammo.drag, ammo.lift)};
    std::cout << "Horizontal flight distance: " << h << " meters" << std::endl;

    float acceleration = pow(drone.attackSpeed, 2) / (2 * drone.accelPath);

    // Out data
    int step = 0;
    float outX[MAX_STEPS + 1];    // xd
    float outY[MAX_STEPS + 1];    // yd
    float outDir[MAX_STEPS + 1];  //
    int outState[MAX_STEPS + 1];  // Dron State
    int outTarget[MAX_STEPS + 1]; // Target index

    // The main loop runs until the drone reaches the drop point or the number of steps exceeds MAX_STEPS.

    float currentTime = 0.0f;

    while (true)
    {
        // Interpolate the positions of all 5 targets.
        float targetX = 0.0f, targetY = 0.0f;
        float ratio = 0.0f;
        float fireX = 0.0f, fireY = 0.0f;
        float distanceToTarget = 0.0f;

        float currentTime = 0.0f;
        float totalTime = 0.0f;
        float minTotalTime = 100000.0f;
        int targetIdx = 0;
        float targetVx = 0.0f;
        float targetVy = 0.0f;

        // For each TARGET_STEPS (60)
        for (int j = 0; j < TARGET_STEPS; j++)
        {
            // For each Target (5)
            for (int i = 0; i < TARGET_COUNT; i++)
            {
                currentTime += drone.arrayTimeStep;

                // Get TargetX, TargetY
                targetX = targetXInTime[i][j];
                targetY = targetYInTime[i][j];

                // Calculate Distance to target
                distanceToTarget = getDistanceToTarget(drone.startPos.x, drone.startPos.y, targetX, targetY);

                // Calculate totalTime (flight time to the drop point)
                totalTime = distanceToTarget / drone.attackSpeed;

                // Calculate Target speed
                float predictedX = getPredictedPosition(targetX, drone.arrayTimeStep, t, totalTime);
                float predictedY = getPredictedPosition(targetY, drone.arrayTimeStep, t, totalTime);

                // Calculate the drop point
                distanceToTarget = getDistanceToTarget(drone.startPos.x, drone.startPos.y, predictedX, predictedY);
                ratio = getRatio(distanceToTarget, h);
                float _fireX = drone.startPos.x + (predictedX - drone.startPos.x) * ratio;
                float _fireY = drone.startPos.y + (predictedY - drone.startPos.y) * ratio;

                // Have dificulties to calculate angularSpeed, but if I do:
                if (drone.angularSpeed > drone.turnThreshold)
                {
                    phase = TURNING;
                    drone.angularSpeed = drone.angularSpeed - drone.turnThreshold;
                }
                else
                {
                    phase = MOVING;
                }

                // Pick the closest target
                if (totalTime < minTotalTime)
                {
                    minTotalTime = totalTime;
                    targetIdx = i;
                    outTarget[step] = targetIdx;
                    outState[step] = phase;
                    outDir[step] = drone.angularSpeed;

                    // Update Drop point for the closest target
                    fireX = _fireX;
                    fireY = _fireY;
                }

                step++;

                // Output" Total time: " << maxTotalTime << std::endl;
            }
            minTotalTime = 100000.0f;
        }

        if (step > MAX_STEPS)
        {
            std::ofstream fout(out_file);
            if (!fout.is_open())
            {
                std::cerr << "Failed to open " << out_file << std::endl;
                return 1;
            }

            json out;
            // Writing N (steps)
            out["totalSteps"] = step;
            // Writing Dron position
            out["steps"] = json::array();
            for (int i = 0; i < step; i++)
            {
                json step;
                step["position"] = {{"x", drone.startPos.x}, {"y", drone.startPos.y}};
                step["direction"] = outDir[i];
                step["state"] = outState[i];
                step["targetIndex"] = outTarget[i];
                // step["dropPoint"] = {{"x", s[i].dropPoint.x},
                //                      {"y", s[i].dropPoint.y}};
                // step["aimPoint"] = {{"x", s[i].aimPoint.x},
                //                     {"y", s[i].aimPoint.y}};
                // step["predictedTarget"] = {{"x", s[i].predictedTarget.x},
                //                            {"y", s[i].predictedTarget.y}};
                out["steps"].push_back(step);
            }

            fout << out.dump(2);
            fout.close();
            break;
        }
    }

    return 0;
}
