#include <iostream>
#include <cmath>
#include <fstream>
#include <cstring>

constexpr char in_file[] = "input.txt";
constexpr char out_file[] = "output.txt";
constexpr float g = 9.81f;

// Ammunition types and their properties
constexpr int AMMO_COUNT = 5;
constexpr char bombNames[AMMO_COUNT][15] = {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
constexpr float bombM[] = {0.35f, 0.6f, 1.2f, 0.45f, 1.4f};    // mass of the ammunition (kg)
constexpr float bombD[] = {0.07f, 0.10f, 0.10f, 0.10f, 0.10f}; // drag coefficient earodynamic resistance
constexpr float bombL[] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f};      // lift coefficient (0 = free fall, 1 = loitering ammunition)

void readInput(float &xd, float &yd, float &zd, float &targetX, float &targetY, float &attackSpeed, float &accelerationPath, char ammo_name[])
{
    std::ifstream input(in_file);
    if (!input.is_open())
    {
        std::cerr << "Failed to open  " << in_file << std::endl;
        return;
    }

    input >> xd >> yd >> zd >> targetX >> targetY >> attackSpeed >> accelerationPath >> ammo_name;
    input.close();
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

double getDistanceToTarget(float xd, float yd, float targetX, float targetY)
{
    return sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));
}

float getIntermediate_coordinate(float vd, float targetV, double distance_to_target, double h, float accelerationPath)
{
    float vp = targetV - (targetV - vd) * (h + accelerationPath) / distance_to_target;
    return vp;
}

double getRatio(double distance_to_target, double h, float accelerationPath)
{
    return (distance_to_target - h) / distance_to_target;
}

int main()
{
    // Drone coordinates (zd - height, m)
    float xd = 0.0f, yd = 0.0f, zd = 0.0f;
    // Target coordinates on the ground (z = 0)
    float targetX = 0.0f, targetY = 0.0f;
    // Drone attack speed (m/s)
    float attackSpeed = 0.0f;
    // Drone acceleration distance before attack (m)
    float accelerationPath = 0.0f;
    // Ammunition name
    char ammo_name[]{};

    // Read input from file
    readInput(xd, yd, zd, targetX, targetY, attackSpeed, accelerationPath, ammo_name);

    std::cout << "Drone coordinates: (" << xd << ", " << yd << ", " << zd << ")" << std::endl;
    std::cout << "Target coordinates: (" << targetX << ", " << targetY << ")" << std::endl;
    std::cout << "Attack speed: " << attackSpeed << " m/s" << std::endl;
    std::cout << "Acceleration  path: " << accelerationPath << " m" << std::endl;
    std::cout << "Ammunition: " << ammo_name << std::endl;

    // Ammunition properties
    float m = 0.0f;
    float d = 0.0f;
    float l = 0.0f;
    // Get ammunition properties
    if (!getAmmunitionProperties(ammo_name, m, d, l))
    {
        return 1;
    }

    // The fall time "t" is determined from the quadratic equation of motion with drag and lift
    // We will use Cardano formula to solve the cubic equation for the time of flight considering drag and lift
    double t{getT(m, d, l, attackSpeed, zd)};
    std::cout << "Flight time: " << t << " seconds" << std::endl;

    // Horisontal flight distance
    double h{getH(t, attackSpeed, d, m, l)};
    std::cout << "Horizontal flight distance: " << h << " meters" << std::endl;

    // Output the firing point to output.txt
    std::ofstream output(out_file);
    if (!output.is_open())
    {
        std::cerr << "Failed to open " << out_file << std::endl;
        return 1;
    }

    // Drop point calculation
    // Step 1: Distance to the target
    double distance_to_target{getDistanceToTarget(xd, yd, targetX, targetY)};
    std::cout << "Distance to target: " << distance_to_target << " meters" << std::endl;

    // Step 2: Necessary maneuvering check
    // If h + accelerationPath < distance_to_target, then the drone needs to maneuver to hit the target.
    if ((h + accelerationPath) > distance_to_target)
    {
        float xp = getIntermediate_coordinate(xd, targetX, distance_to_target, h, accelerationPath);
        float yp = getIntermediate_coordinate(yd, targetY, distance_to_target, h, accelerationPath);

        std::cout << "The drone needs to maneuver to hit the target." << std::endl;
        std::cout << "New drone coordinates for attack: (" << xp << ", " << yp << ", " << zd << ")" << std::endl;
        output << xp << " " << yp << " ";
    }

    // Step 3: Calculate the firing point
    double ratio{getRatio(distance_to_target, h, accelerationPath)};
    double fireX = xd + (targetX - xd) * ratio;
    double fireY = yd + (targetY - yd) * ratio;

    output << fireX << " " << fireY << std::endl;
    output.close();

    return 0;
}
