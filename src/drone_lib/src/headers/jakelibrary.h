
#ifndef JAKELIBRARY_H
#define JAKELIBRARY_H

#include <iostream>
#include <cmath>
#include <math.h>

    // Initialising
    const float switchDist = 1.0; // In reality, camera should start working when it is 8m away. However, GPS is on car NOT THE PLATFORM so probably 6.5 m max
    float distance;
    float droneVel[3] = {0}; // Relative to earth
    float relVel[3] = {0};
    float droneAcc[3] = {0};
    float relPos[3] = {0};
    float relPosOld[3] = {0};
    float relPosHoriz[3] = {0};

    // declare functions
    void droneInitVel(float relPos[3], float (&droneVel)[3]); 
    void InitialiseJakeCode(float x, float y, float z);
    void cross(float vector1[3], float vector2[3], float (&xproduct)[3]);
    float norm(float vector[3]);
    void velFromGPS(float relPos[3], float relPosOld[3], float loop_rate, float (&relVel)[3]);
    float dot(float vector1[3], float vector2[3]);
    void droneAccComp(float relPos[3], float relVel[3], float (&droneAcc)[3]);

    ///< testfix
    double velocityholder[3];

    // Landing
    void velPosMap(float relPosLanding[3], float (&relVelLanding)[3]);



//< ------------------------------------ Function Definitions ------------------------------------------>

///<Jake Code Initialisation
// INPUT:  target position position or relative position (its the same), NED
// OUTPUT: runs droneInitVel,
//         calculates distance to target using norm
void InitialiseJakeCode(float x, float y, float z){ ///< initialise with arguments of relative position of drone

    relPos[0] = y;
    relPos[1] = x;
    relPos[2] = z;
    relPos[2] = 0.0;

    droneInitVel(relPos, droneVel);
    relVel[0] = -droneVel[0];
    relVel[1] = -droneVel[1];   
    relVel[2] = -droneVel[2];

    distance = norm(relPos);

}

// Calculates an initial drone velocity which is in the direction of the target.
// Passes the output, droneVel, by reference
// NOTE: If target is precisely East or West of drone, e.g. dronePos[0] = targPos[0], the algorithm follows a strange
//       trajectory, however it will still reach the target.
// INPUT:  relative position, NED
// OUTPUT: the required drone velocity in the direction of the target
void droneInitVel(float relPos[3], float (&droneVel)[3]) {

    float heading;
    // This may need to be changed depending on what is a reasonable velocity for the drone to travel at
    // Obviously need to take into account the time it will take for the drone to reach this velocity
    // How fast can the drone accelerate?
    float initVelMag = 2;

    // relPosCalc(dronePos, targPos, relPos);

    // Angle to target
    heading = atan(relPos[1] / relPos[0]);

    // Split up the initial velocity into its components in the North and East directions
    droneVel[0] = initVelMag * cos(heading);
    droneVel[1] = initVelMag * sin(heading);
    droneVel[2] = 0;

    // Make sure the velocities are in the right direction
    if (relPos[0] < 0) {
        droneVel[0] = - droneVel[0];
        droneVel[1] = - droneVel[1];
    }
}


// Computes the cross product of a 3-dimensional vector
// Passes the output, xproduct, by reference
void cross(float vector1[3], float vector2[3], float (&xproduct)[3]) {
    float i =   vector1[1] * vector2[2] - vector1[2] * vector2[1];
    float j = -(vector1[0] * vector2[2] - vector1[2] * vector2[0]);
    float k =   vector1[0] * vector2[1] - vector1[1] * vector2[0];
    xproduct[0] = i;
    xproduct[1] = j;
    xproduct[2] = k;
}

// Computes the norm of a 3-dimensional vector
float norm(float vector[3]) {

    float accum = 0;

    for (int i = 0; i < 3; ++i) {
        accum += vector[i] * vector[i];
    }

    float norm = sqrt(accum);

    return norm;
}

// Computes the dot product of two vectors
float dot(float vector1[3], float vector2[3]) {

    float accum = 0;

    for (int i = 0; i < 3; ++i) {
        accum += vector1[i] * vector2[i];
    }

    return accum;
}


// INPUTS:  A position for this and last time step. Can be taregt, drone or relative. Need previous time step, NED
// OUTPUTS: velocity corresponding to whatever position was input, NED
void velFromGPS(float relPos[3], float relPosOld[3], float loop_rate, float (&vel)[3]) {

if (relPos != relPosOld){  ///< if not the same position, update velocity
    for (int i = 0; i < 3; ++i) {
        velocityholder[i] = ( relPos[i] - relPosOld[i] ) / (1.0 / loop_rate);
    }
}
vel[0] = velocityholder[0];
vel[1] = velocityholder[1];
vel[2] = velocityholder[2];


}


// Calculates the acceleration of the drone in i,j,k
// Passes the output, droneAcc by reference
// INPUT:  relative position, NED, relative velocity, NED
// OUTPUT: drone acceleration, NED.
void droneAccComp(float relPos[3], float relVel[3], float (&droneAcc)[3]) {

    float  uMag;
    float  uDotMag;
    float  numOmega[3];
    float  denOmega;
    float  Omega[3];
    float  lambda = 3.0;
    float  Kd;
    float  Kp;
    float  uCrossOmega[3];
    float  aPerp[3];
    float  aParr[3];
    float  acclim = 2.0;

    relPos[2] = 0.0;
    // Calculate the magnitude of the relative position and velocity vectors
    uMag    = norm(relPos);
    uDotMag = norm(relVel);

    // ALGORITHM START
    // Autonomous Landing of a Multirotor Micro Air Vehicle on a High Velocity Ground Vehicle
    // Alexandre Borowcyzk

    cross(relPos, relVel, numOmega);

    denOmega = dot(relPos, relPos);

    for (int i = 0; i < 3; ++i) {
        Omega[i] = numOmega[i] / denOmega;
    }

    // Controller gains
    Kd = -lambda * dot(relPos, relVel) / uMag / uMag;
    Kp = lambda * uDotMag * uDotMag / uMag / uMag;

    cross(relPos, Omega, uCrossOmega);

    for (int i = 0; i < 3; ++i) {
        aPerp[i] = - lambda * uDotMag / uMag * uCrossOmega[i];
        aParr[i] = Kp * relPos[i] + Kd * relVel[i];
        droneAcc[i] = aPerp[i] + aParr[i];
    }

    // If the total acceleration is greater than 2, scale each component such that the norm is 2
    if ( norm(droneAcc) > acclim ) {
        float normDroneAcc = norm(droneAcc);
        for (int i = 0; i < 3; ++i) {
            droneAcc[i] /= (normDroneAcc / acclim);
        }
    }
}


// Linear mapping between velocity and position. Basically it takes the relative position, 
// and gives the drone a relative velocity in the same direction but with two thirds the magnitude.
// If the position exceeds three metres, the velocity is capped at 2.
// INPUTS:  the relative position between the drone and the target, NED
// OUTPUTS: the relative velocity between the drone and the target, NED
void velPosMap(float relPosLanding[3], float (&relVelLanding)[3]) {
    float velMax = 2.0;
    float posMax = 3.0;
    float distance;
    float scaling;

    distance = norm(relPosLanding);

    for (int i = 0; i < 3; ++i) {
        relVelLanding[i] = relPosLanding[i];
    }

    if (distance > posMax) {
        for (int i = 0; i < 3; ++i) {
            relVelLanding[i] /= ( distance / velMax );
        }
    }
    else {
        scaling = norm(relPosLanding) * velMax / posMax;
        for (int i = 0; i < 3; ++i) {
            relVelLanding[i] /= ( distance / scaling );
        }
    }
}




#endif