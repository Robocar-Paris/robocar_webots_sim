#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <cmath>
#include <iostream>

using namespace webots;

#define MAX_SPEED 10.0
#define TARGET_ERROR 0.05 

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  // --- 1. MOTEURS ---
  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }

  // --- 2. CAPTEURS ---
  GPS *gps = robot->getGPS("gps");
  gps->enable(timeStep);

  Compass *compass = robot->getCompass("compass");
  compass->enable(timeStep);

  // --- 3. CIBLE ---
  double target_x = 2.0; 
  double target_z = -1.5; 

  std::cout << "Navigation vers : " << target_x << ", " << target_z << std::endl;

  while (robot->step(timeStep) != -1) {
    const double *pos = gps->getValues();
    const double *north = compass->getValues();

    if (std::isnan(pos[0]) || std::isnan(north[0])) continue;

    // --- LOGIQUE DE NAVIGATION ---
    
    // 1. Calcul de l'angle du robot (Heading) par rapport à l'axe X (Nord/Devant)
    // north[0] est X, north[2] est Z.
    // L'angle du vecteur nord dans le repère robot nous donne l'inverse de la rotation du robot
    double robot_heading = atan2(north[0], north[2]); 
    // Note : Selon l'orientation de Webots, parfois c'est atan2(x, z) ou atan2(z, x). 
    // Si le robot tourne en rond, essayez d'inverser les paramètres ici.

    // 2. Calcul de l'angle vers la cible
    double delta_x = target_x - pos[0];
    double delta_z = target_z - pos[2];
    double target_angle = atan2(delta_x, delta_z);

    // 3. Erreur
    double error = normalizeAngle(target_angle - robot_heading);
    double dist = sqrt(delta_x*delta_x + delta_z*delta_z);

    double left_speed = 0;
    double right_speed = 0;

    if (dist < TARGET_ERROR) {
       std::cout << "ARRIVED" << std::endl;
       left_speed = 0;
       right_speed = 0;
       // On peut arrêter la simulation ou juste stopper les moteurs
    } 
    else {
       // Correction proportionnelle
       // Si error > 0, la cible est à gauche -> on tourne à gauche (roue droite + vite)
       // Attention : vérifiez le signe selon votre géométrie
       double turn = error * 5.0; 
       
       // Base speed
       double forward = MAX_SPEED;
       
       // Si l'erreur est grande, on ralentit pour mieux tourner
       if(fabs(error) > 0.5) forward = 2.0;

       left_speed = forward + turn;
       right_speed = forward - turn;
    }

    // Clamp speeds
    if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
    if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

    wheels[0]->setVelocity(left_speed);
    wheels[2]->setVelocity(left_speed);
    wheels[1]->setVelocity(right_speed);
    wheels[3]->setVelocity(right_speed);
  };

  delete robot;
  return 0;
}