#include "Misc.h"
#include "Constants.h"
#include <math.h>
#include <stdio.h>
#include <sys/time.h>

double misc::rpmToTalonVel(double rpm) {
    return (rpm * MotorConstants::kFalconUnitsPerRev) / 600.0;
}

double misc::talonVelToRpm(double units) {
    return (units * 600.0) / MotorConstants::kFalconUnitsPerRev;
}

double misc::degToRad(double degrees) {
    return degrees * 2 * M_PI / 360.0;
}

double misc::radToDeg(double radians) {
    return radians * 360 / (2 * M_PI);
}

double misc::clampDegrees(double degrees) {
    while (degrees > 360 || degrees < 0)
    {
        double out = 360 - degrees;
        if (out < 0) {
            degrees = degrees - 360;
        }
        if (out > 360) {
            degrees = degrees + 360;
        }
    }
    return degrees;
}

double misc::clampRotations(double rotations) {
    double out = fmod(rotations, 1);
    if (out < 0) {
        return 1 + out;
    } else {
        return out;
    }
}

double misc::degreesDiff(double a, double b) {
    a = clampDegrees(a);
    b = clampDegrees(b);

    double diff = fabs(b - a);
    if (diff > 180) {
        return 360 - diff;
    } else {
        return diff;
    }
}

double misc::getSeconds() {
    // return time in seconds as a double
    timeval tv0;
    gettimeofday(&tv0, nullptr);
    return 1.0 * tv0.tv_sec + (1.0 * tv0.tv_usec) / 1000000.0;
}

//DEFINES IDS FOR SWERVE, ALLOWING FOR EASY CODE SWITCHING OF IDS
/*

Module Numbering Scheme:

m = number engraved on module

    - Drive ID: 2m - 1
    - Steering ID: 2m
    - Encoder ID: 16 + m

*/

int misc::GetFLDrive() {
    int trueID = 2 * SwerveConstants::FL_ID - 1;
    return trueID;
}

int misc::GetFLSteer() {
    int trueID = 2 * SwerveConstants::FL_ID;
    return trueID;
}

int misc::GetFLCANCoder() {
    int trueID = 16 + SwerveConstants::FL_ID;
    return trueID;
}

int misc::GetFRDrive() {
    int trueID = 2 * SwerveConstants::FR_ID - 1;
    return trueID;
}

int misc::GetFRSteer() {
    int trueID = 2 * SwerveConstants::FR_ID;
    return trueID;
}

int misc::GetFRCANCoder() {
    int trueID = 16 + SwerveConstants::FR_ID;
    return trueID;
}

int misc::GetBLDrive(){
    int trueID = 2 * SwerveConstants::BL_ID - 1;
    return trueID;
}

int misc::GetBLSteer(){
    int trueID = 2 * SwerveConstants::BL_ID;
    return trueID;
}

int misc::GetBLCANCoder(){
    int trueID = 16 + SwerveConstants::BL_ID;
    return trueID;
}

int misc::GetBRDrive() {
    int trueID = 2 * SwerveConstants::BR_ID - 1;
    return trueID;
}

int misc::GetBRSteer() {
    int trueID = 2 * SwerveConstants::BR_ID;
    return trueID;
}

int misc::GetBRCANCoder() {
    int trueID = 16 + SwerveConstants::BR_ID;
    return trueID;
}

double misc::gettime_d(){
	// return time in seconds as a double
	double t0;
	struct timeval tv0;

	gettimeofday(&tv0, NULL);
	t0 = 1.0 * tv0.tv_sec + (1.0 * tv0.tv_usec) / 1000000.0;
	// printf("seconds: %ld\n", tv0.tv_sec);
	// printf("usecs:   %ld\n", tv0.tv_usec);
	// printf("time:    %lf\n", t0);

	return t0;
}