#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>

// Ammunition types and their properties
constexpr char in_file[] = "input.txt";
constexpr char targets_file[] = {"targets.txt"};
constexpr char out_file[] = "simulation.txt";

// Ammunition types and their properties
constexpr int AMMO_COUNT = 5;
constexpr int TARGET_COUNT = 5;
constexpr int TARGET_STEPS = 60;
constexpr char bombNames[AMMO_COUNT][15] = {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
constexpr float bombM[] = {0.35f, 0.6f, 1.2f, 0.45f, 1.4f};    // mass of the ammunition (kg)
constexpr float bombD[] = {0.07f, 0.10f, 0.10f, 0.10f, 0.10f}; // drag coefficient earodynamic resistance
constexpr float bombL[] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f};      // lift coefficient (0 = free fall, 1 = loitering ammunition)

// Input parameters
// Drone coordinates (zd - height, m)
float xd = 0.0f, yd = 0.0f, zd = 0.0f;
// Initial drone direction
float initialDir = 0.0f;
// Drone attack speed (m/s)
float attackSpeed = 0.0f;
// Drone acceleration distance before attack (m)
float accelerationPath = 0.0f;
// Ammunition name
char ammo_name[] = "Unknown";
// Time step for Target coordinates array (sec)
float arrayTimeStep = 0.0f;
// Time step for Simulation (sec)
float simTimeStep = 0.0f;
// Kill zone radius - allowed hit error (m)
float hitRadius = 0.0f;
// Angular spped of rotation (rad/sec)
float angularSpeed = 0.0f;
// Threshold angle for stopping
float turnThreshold = 0.0f;

// Drone states
enum DroneState
{
    STOPPED,
    ACCELERATING,
    DECELERATING,
    TURNING,
    MOVING,
};

constexpr float g = 9.81f;
constexpr int MAX_STEPS = 10000;

void readInput(float &xd, float &yd, float &zd, float &initialDir, float &attackSpeed, float &accelerationPath, char ammo_name[], float &arrayTimeStep, float &simTimeStep, float &hitRadius, float &angularSpeed, float &turnThreshold)
{
    std::ifstream input(in_file);
    if (!input.is_open())
    {
        std::cerr << "Failed to open  " << in_file << std::endl;
        return;
    }

    input >> xd >> yd >> zd >> initialDir >> attackSpeed >> accelerationPath >> ammo_name >> arrayTimeStep >> simTimeStep >> hitRadius >> angularSpeed >> turnThreshold;
    input.close();
}

void readTargets(float targetXInTime[TARGET_COUNT][TARGET_STEPS], float targetYInTime[TARGET_COUNT][TARGET_STEPS])
{
    std::ifstream targetFile(targets_file);
    if (!targetFile.is_open())
    {
        std::cerr << "Failed to open  " << targets_file << std::endl;
        return;
    }
    for (int i = 0; i < TARGET_COUNT; i++)
    {
        for (int j = 0; j < TARGET_STEPS; j++)
        {
            targetFile >> targetXInTime[i][j];
        }
    }
    for (int i = 0; i < TARGET_COUNT; i++)
    {
        for (int j = 0; j < TARGET_STEPS; j++)
        {
            targetFile >> targetYInTime[i][j];
        }
    }
    targetFile.close();
}

bool getAmmunitionProperties(const char ammo_name[], float &m, float &d, float &l)
{
    for (int i = 0; i < AMMO_COUNT; i++)
    {
        if (strcmp(ammo_name, bombNames[i]) == 0)
        {
            m = bombM[i];
            d = bombD[i];
            l = bombL[i];
            return true;
        }
    }
    std::cout << "Unknown ammo\n";
    return false;
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

void interpolateTarget(int targetIdx, float t, float arrayTimeStep, float &outTx, float &outTy, float targetXInTime[TARGET_COUNT][TARGET_STEPS], float targetYInTime[TARGET_COUNT][TARGET_STEPS])
{
    int idx = static_cast<int>(floor(t / arrayTimeStep) / arrayTimeStep) % TARGET_STEPS;
    int next = (idx + 1) % TARGET_STEPS;
    float frac = (t - idx * arrayTimeStep) / arrayTimeStep;
    outTx = targetXInTime[targetIdx][idx] + (targetXInTime[targetIdx][next] - targetXInTime[targetIdx][idx]) * frac;
    outTy = targetYInTime[targetIdx][idx] + (targetYInTime[targetIdx][next] - targetYInTime[targetIdx][idx]) * frac;
}
float getDronPosition(float targetP, float simTimeStep, float t)
{
    float dp = targetP * (t + simTimeStep) - targetP * (t);

    return dp;
}

float getPredictedPosition(float dp, float targetP, float simTimeStep, float t, float totalTime)
{
    float targetVp = dp / simTimeStep;
    float predictedP = targetP * (t) + targetVp * totalTime;

    return predictedP;
}

double getDistanceToTarget(float xd, float yd, float targetX, float targetY)
{
    return sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));
}

double getRatio(double distance_to_target, double h, float accelerationPath)
{
    return (distance_to_target - h) / distance_to_target;
}

float getTimeToStop(DroneState phase, float acceleration)
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
        return attackSpeed / acceleration;

    default:
        return 0;
    }
}

int main()
{
    // Read input parameters from file
    readInput(xd, yd, zd, initialDir, attackSpeed, accelerationPath, ammo_name, arrayTimeStep, simTimeStep, hitRadius, angularSpeed, turnThreshold);

    std::cout << "Input Data:" << std::endl;
    std::cout << "Drone coordinates: (" << xd << ", " << yd << ", " << zd << ")" << std::endl;
    std::cout << "Initial drone direction: " << initialDir << " degrees" << std::endl;
    std::cout << "Attack speed: " << attackSpeed << " m/s" << std::endl;
    std::cout << "Acceleration  path: " << accelerationPath << " m" << std::endl;
    std::cout << "Ammunition: " << ammo_name << std::endl;
    std::cout << "Time step for Target coordinates array: " << arrayTimeStep << " sec" << std::endl;
    std::cout << "Time step for Simulation: " << simTimeStep << " sec" << std::endl;
    std::cout << "Kill zone radius: " << hitRadius << " m" << std::endl;
    std::cout << "Angular speed of rotation: " << angularSpeed << " rad/sec" << std::endl;
    std::cout << "Threshold angle for stopping: " << turnThreshold << " degrees" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    // Ammunition properties
    float m = 0.0f;
    float d = 0.0f;
    float l = 0.0f;
    // Get ammunition properties
    if (!getAmmunitionProperties(ammo_name, m, d, l))
    {
        return 1;
    }

    std::cout << "Ammunition properties: " << std::endl;
    std::cout << "Mass: " << m << " kg" << std::endl;
    std::cout << "Drag coefficient: " << d << std::endl;
    std::cout << "Lift coefficient: " << l << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    // Drone state
    DroneState phase = STOPPED;

    // Read target coordinates from file
    // Target coordinates
    float targetXInTime[TARGET_COUNT][TARGET_STEPS];
    float targetYInTime[TARGET_COUNT][TARGET_STEPS];

    readTargets(targetXInTime, targetYInTime);

    // The fall time "t" is determined from the quadratic equation of motion with drag and lift
    // We will use Cardano formula to solve the cubic equation for the time of flight considering drag and lift
    double t{getT(m, d, l, attackSpeed, zd)};
    std::cout << "Flight time: " << t << " seconds" << std::endl;

    // Horisontal flight distance
    double h{getH(t, attackSpeed, d, m, l)};
    std::cout << "Horizontal flight distance: " << h << " meters" << std::endl;

    float acceleration = pow(attackSpeed, 2) / (2 * accelerationPath);

    // Out data
    float outX[MAX_STEPS + 1];    // xd
    float outY[MAX_STEPS + 1];    // yd
    float outDir[MAX_STEPS + 1];  //
    int outState[MAX_STEPS + 1];  // Dron State
    int outTarget[MAX_STEPS + 1]; // Target index

    // The main loop runs until the drone reaches the drop point or the number of steps exceeds MAX_STEPS.
    int step = 0;
    float currentTime = 0.0f;

    while (true)
    {
        // Interpolate the positions of all 5 targets.
        float outTx = 0.0f, outTy = 0.0f;
        float maxTotalTime[TARGET_COUNT] = {};
        float totalTime = 0.0f;

        for (int i = 0; i < TARGET_COUNT; i++)
        {
            phase = MOVING;

            interpolateTarget(i, totalTime, arrayTimeStep, outTx, outTy, targetXInTime, targetYInTime);

            // Dron Position
            outX[step] = getDronPosition(xd, simTimeStep, currentTime);
            outY[step] = getDronPosition(yd, simTimeStep, currentTime);

            // Lead targeting
            float predictedX = getPredictedPosition(outX[step], outTx, simTimeStep, currentTime, totalTime);
            float predictedY = getPredictedPosition(outY[step], outTy, simTimeStep, currentTime, totalTime);

            // Drop point calculation
            // Step 1: Distance to the target
            double distance_to_target{getDistanceToTarget(xd, yd, predictedX, predictedY)};

            // Step 2: Calculate the firing point
            double ratio{getRatio(distance_to_target, h, accelerationPath)};
            double fireX = xd + (predictedX - xd) * ratio;
            double fireY = yd + (predictedY - yd) * ratio;

            // totalTime (Dron to Target flight)
            float timeToStop = getTimeToStop(phase, acceleration);
            totalTime += timeToStop;

            currentTime += simTimeStep;
            totalTime += simTimeStep;
            totalTime += currentTime;
            std::cout << "Total time: " << totalTime << std::endl;
            std::cout << "Time to stop: " << timeToStop << std::endl;
            step++;
        }

        if (step > MAX_STEPS)
        {
            break;
        }
    }

    return 0;
}
