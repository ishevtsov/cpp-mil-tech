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

void readIn(float &xd, float &yd, float &zd, float &initialDir, float &attackSpeed, float &accelerationPath, char ammo_name[], float &arrayTimeStep, float &simTimeStep, float &hitRadius, float &angularSpeed, float &turnThreshold)
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
// Writing Float Array data to the file
void writeArrOut(float data[MAX_STEPS + 1], int step, std::ofstream &output)
{
    for (int i = 0; i < step; i++)
    {
        output << data[i] << " ";
    }
    output << std::endl;
}

// Writing Int Array data to the file
void writeIntOut(int data[MAX_STEPS + 1], int step, std::ofstream &output)
{
    for (int i = 0; i < step; i++)
    {
        output << data[i] << " ";
    }
    output << std::endl;
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
    readIn(xd, yd, zd, initialDir, attackSpeed, accelerationPath, ammo_name, arrayTimeStep, simTimeStep, hitRadius, angularSpeed, turnThreshold);

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
    int step = 0;
    float outX[MAX_STEPS + 1];    // xd
    float outY[MAX_STEPS + 1];    // yd
    float outDir[MAX_STEPS + 1];  //
    int outState[MAX_STEPS + 1];  // Dron State
    int outTarget[MAX_STEPS + 1]; // Target index

    // The main loop runs until the drone reaches the drop point or the number of steps exceeds MAX_STEPS.

    float currentTime = 0.0f;

    std::ofstream output(out_file);
    if (!output.is_open())
    {
        std::cerr << "Failed to open " << out_file << std::endl;
        return 1;
    }

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
                currentTime += arrayTimeStep;

                // Get TargetX, TargetY
                targetX = targetXInTime[i][j];
                targetY = targetYInTime[i][j];

                // Calculate Distance to target
                distanceToTarget = getDistanceToTarget(xd, yd, targetX, targetY);

                // Calculate totalTime (flight time to the drop point)
                totalTime = distanceToTarget / attackSpeed;

                // Calculate Target speed
                float predictedX = getPredictedPosition(targetX, arrayTimeStep, t, totalTime);
                float predictedY = getPredictedPosition(targetY, arrayTimeStep, t, totalTime);

                // Calculate the drop point
                distanceToTarget = getDistanceToTarget(xd, yd, predictedX, predictedY);
                ratio = getRatio(distanceToTarget, h);
                float _fireX = xd + (predictedX - xd) * ratio;
                float _fireY = yd + (predictedY - yd) * ratio;

                // Have dificulties to calculate angularSpeed, but if I do:
                if (angularSpeed > turnThreshold)
                {
                    phase = TURNING;
                    angularSpeed = angularSpeed - turnThreshold;
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
                    outDir[step] = angularSpeed;

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
            // Writing N (steps)
            output << step << std::endl;
            // Writing Dron position
            float outXY[MAX_STEPS * 2 + 1];
            int index = 0;
            for (int i = 0; i < MAX_STEPS; ++i)
            {
                outXY[index++] = outX[i];
                outXY[index++] = outY[i];
            }
            writeArrOut(outXY, step, output);
            // Writing Direction data
            writeArrOut(outDir, step, output);
            // Writing Dron State
            writeIntOut(outState, step, output);
            // Writing Target index
            writeIntOut(outTarget, step, output);

            output.close();
            break;
        }
    }

    return 0;
}
