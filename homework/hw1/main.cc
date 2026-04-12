#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>

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
    char ammo_name[] = "Unknown";

    // Read input from file
    std::ifstream input("input.txt");
    if (!input.is_open())
    {
        std::cerr << "Failed to open input.txt" << std::endl;
        return 1;
    }

    input >> xd >> yd >> zd >> targetX >> targetY >> attackSpeed >> accelerationPath >> ammo_name;
    input.close();

    std::cout << "Drone coordinates: (" << xd << ", " << yd << ", " << zd << ")" << std::endl;
    std::cout << "Target coordinates: (" << targetX << ", " << targetY << ")" << std::endl;
    std::cout << "Attack speed: " << attackSpeed << " m/s" << std::endl;
    std::cout << "Acceleration  path: " << accelerationPath << " m" << std::endl;
    std::cout << "Ammunition: " << ammo_name << std::endl;

    // Ammunition types and their properties
    float m = 0.0f; // mass of the ammunition (kg)
    float d = 0.0f; // drag coefficient earodynamic resistance
    float l = 0.0f; // lift coefficient (0 = free fall, 1 = loitering ammunition)

    if (strcmp(ammo_name, "VOG-17") == 0)
    {
        m = 0.35f;
        d = 0.07f;
        l = 0.0f;
    }
    else if (strcmp(ammo_name, "M67") == 0)
    {
        m = 0.6f;
        d = 0.1f;
        l = 0.0f;
    }
    else if (strcmp(ammo_name, "RKG-3") == 0)
    {
        m = 1.2f;
        d = 0.1f;
        l = 0.0f;
    }
    else if (strcmp(ammo_name, "GLIDING-VOG") == 0)
    {
        m = 0.45f;
        d = 0.1f;
        l = 1.0f;
    }
    else if (strcmp(ammo_name, "GLIDING-RKG") == 0)
    {
        m = 1.4f;
        d = 0.1f;
        l = 1.0f;
    }
    else
    {
        std::cout << "Unknown ammo\n";
        return 1;
    }

    // Calculation of the Flight time
    double g = 9.81;

    // Coefficients for the quadratic equation of motion with drag and lift
    // We will use Cardano formula to solve the cubic equation for the time of flight considering drag and lift
    float a = d * g * m - 2 * pow(d, 2) * l * attackSpeed;
    float b = -3 * g * pow(m, 2) + 3 * d * l * m * attackSpeed;
    float c = 6 * pow(m, 2) * zd;

    float p = -1 * pow(b, 2) / (3 * pow(a, 2));
    float q = 2 * pow(b, 3) / (27 * pow(a, 3)) + c / a;

    float phi = std::acos(3 * q / (2 * p) * std::sqrt(-3 / p));

    float t = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * M_PI) / 3) - b / (3 * a);
    std::cout << "Flight time: " << t << " seconds" << std::endl;

    // Horisontal flight distance
    float h = attackSpeed * t - pow(t, 2) * d * attackSpeed / (2 * m) + pow(t, 3) * (6 * d * g * l * m - 6 * pow(d, 2) * (pow(l, 2) - 1) * attackSpeed) / (36 * pow(m, 2)) +
              pow(t, 4) * (-6 * pow(d, 2) * g * l * (1 + pow(l, 2) + pow(l, 4)) * m + 3 * pow(d, 3) * pow(l, 2) * (1 + pow(l, 2)) * attackSpeed + 6 * pow(d, 3) * pow(l, 4) * (1 + pow(l, 2)) * attackSpeed) / (pow((36 * (1 + pow(l, 2))), 2) * pow(m, 3)) + pow(t, 5) * (3 * pow(d, 3) * g * pow(l, 3) * m - 3 * pow(d, 4) * pow(l, 2) * (1 + pow(l, 2)) * attackSpeed) / (36 * (1 + pow(l, 2)) * pow(m, 4));

    std::cout << "Horizontal flight distance: " << h << " meters" << std::endl;

    // Drop point calculation
    // Step 1: Distance to the target
    float distance_to_target = sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));
    std::cout << "Distance to target: " << distance_to_target << " meters" << std::endl;

    // Output the firing point to output.txt
    std::ofstream output("output.txt");
    if (!output.is_open())
    {
        std::cerr << "Failed to open output.txt" << std::endl;
        return 1;
    }

    // Step 2: Necessary maneuvering check
    // If h + accelerationPath < distance_to_target, then the drone needs to maneuver to hit the target.
    float fireX = 0.0f, fireY = 0.0f;

    if ((h + accelerationPath) < distance_to_target)
    {
        xd = targetX - (targetX - xd) * (h + accelerationPath) / distance_to_target;
        yd = targetY - (targetY - yd) * (h + accelerationPath) / distance_to_target;

        std::cout << "The drone needs to maneuver to hit the target." << std::endl;
        std::cout << "New drone coordinates for attack: (" << xd << ", " << yd << ", " << zd << ")" << std::endl;
        output << xd << " " << yd << " ";
    }

    // Step 3: Calculate the firing point
    float ratio = (distance_to_target - h) / distance_to_target;
    fireX = xd + (targetX - xd) * ratio;
    fireY = yd + (targetY - yd) * ratio;

    output << fireX << " " << fireY << std::endl;
    output.close();

    return 0;
}
