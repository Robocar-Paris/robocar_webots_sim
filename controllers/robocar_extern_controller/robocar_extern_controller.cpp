#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>

using namespace webots;
using namespace std;

int main() {
    Robot *robot = new Robot();
    int timeStep = 32;

    Motor *wheel1 = robot->getMotor("wheel1");
    Motor *wheel2 = robot->getMotor("wheel2");
    Motor *wheel3 = robot->getMotor("wheel3");
    Motor *wheel4 = robot->getMotor("wheel4");
    
    if (!wheel1 || !wheel2 || !wheel3 || !wheel4) {
        cerr << "ERROR: Failed to get motors!" << endl;
        return 1;
    }
    
    wheel1->setPosition(INFINITY);
    wheel2->setPosition(INFINITY);
    wheel3->setPosition(INFINITY);
    wheel4->setPosition(INFINITY);

    wheel1->setVelocity(0.0);
    wheel2->setVelocity(0.0);
    wheel3->setVelocity(0.0);
    wheel4->setVelocity(0.0);
    
    cout << "Motors initialized!" << endl;
    
    // Rendre stdin non-bloquant
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    
    string buffer = "";

    float angularFactor = 1;
    float maxSpeed = 15.0;
    
    cout << "Ready to receive commands (format: linear,angular)" << endl;
    
    // Boucle principale
    while (robot->step(timeStep) != -1) {
        char tmp[256];
        ssize_t n = read(STDIN_FILENO, tmp, sizeof(tmp) - 1);
        
        if (n > 0) {
            tmp[n] = '\0';
            buffer += tmp;
            size_t pos;
            
            while ((pos = buffer.find('\n')) != string::npos) {
                string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);

                float linear, angular;
                if (sscanf(line.c_str(), "%f,%f", &linear, &angular) == 2) {
                    float angularAdjusted = -angular * angularFactor;
                    
                    float leftSpeed = linear + angularAdjusted;
                    float rightSpeed = linear - angularAdjusted;
                    
                    leftSpeed = max(-maxSpeed, min(maxSpeed, leftSpeed));
                    rightSpeed = max(-maxSpeed, min(maxSpeed, rightSpeed));
                    
                    wheel1->setVelocity(leftSpeed);
                    wheel3->setVelocity(leftSpeed);
                    wheel2->setVelocity(rightSpeed);
                    wheel4->setVelocity(rightSpeed);
                    
                    cout << "CMD: linear=" << linear 
                         << " angular=" << angular 
                         << " -> L=" << leftSpeed 
                         << " R=" << rightSpeed << endl;
                } else {
                    cerr << "ERROR: Invalid command format: " << line << endl;
                }
            }
        }
    }
    
    wheel1->setVelocity(0.0);
    wheel2->setVelocity(0.0);
    wheel3->setVelocity(0.0);
    wheel4->setVelocity(0.0);
    
    delete robot;
    cout << "Controller shutdown" << endl;
    return 0;
}
