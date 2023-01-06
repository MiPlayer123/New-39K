#include "odometry.h"

//CONSTANTS / Hard-Coded Values
//Radius of tracking wheels in inches
double WHEEL_RADIUS = 3.25/2;
double SWHEEL_RADIUS = 2.80/2;

//Starting angle (relative to field) (RADIANS)
double THETA_START = THETA_START; //imagine the field is a unit circle

//The starting x and y coordinates of the bot (INCHES)
  //These distances are relative to some point (0,0) on the field
  //Relative to: BOTTOM LEFT CORNER
double X_START = X_START; //19.1
double Y_START = Y_START; //8.5

//Distances of tracking wheels from tracking center (INCHES)
double LTrackRadius = 5.875; //was 6.87
double RTrackRadius = 5.875; //6.8335
double STrackRadius = 3.75;

//Calculated Values (every loop)
//Angles (DEGREES) *NEEDS TO BE CONVERTED TO RADIANS FOR MATH*
double LPos = 0;
double RPos = 0;
double SPos = 0;

double LPrevPos = 0;
double RPrevPos = 0;
double SPrevPos = 0;

//Distances traveled by tracking wheels each loop (INCHES)
double deltaDistL = 0;
double deltaDistR = 0;
double deltaDistS = 0;

//Distance summations (since last reset)
double totalDeltaDistL = 0;
double totalDeltaDistR = 0;

//The current angle of the bot (RADIANS)
double currentAbsoluteOrientation = THETA_START;
//The previous angle of the bot (RADIANS)
double previousTheta = THETA_START;

//The change in Theta each loop (RADIANS)
double deltaTheta = 0;

//The Average angle Theta (In RADIANS) throughout the arc
  //currentAbsoluteOrientation + (deltaTheta / 2)
double avgThetaForArc = currentAbsoluteOrientation + (deltaTheta / 2);

//The changes in the X and Y positions (INCHES)
/*These are calculated on a local basis each loop,
then converted to global position changes */
double deltaXLocal = 0;
double deltaYLocal = 0;

//The X and Y offsets converted from their local forms (INCHES)
double deltaXGlobal = 0;
double deltaYGlobal = 0;

//The global position of the bot (INCHES)
double xPosGlobal = X_START;
double yPosGlobal = Y_START;


int positionTracking() {
  while(1) {
    //Get encoder values (DEGREES)
    
    LPos = (BaseLeftFront.position(deg) + BaseLeftMid.position(deg) + BaseLeftRear.position(deg))/3.0;//LTrack.position(rotationUnits::deg);
    RPos = (BaseRightFront.position(deg) + BaseRightMid.position(deg) + BaseRightRear.position(deg))/3.0;//RTrack.position(rotationUnits::deg);
    SPos = STrack.position(rotationUnits::deg);

    //Calculate distance traveled by tracking each wheel (INCHES)
      //Converts degrees to radians
    deltaDistL = ((LPos - LPrevPos) * M_PI / 180) * WHEEL_RADIUS * (3.0/5.0);
    deltaDistR = ((RPos - RPrevPos) * M_PI / 180) * WHEEL_RADIUS * (3.0/5.0);
    deltaDistS = ((SPos - SPrevPos) * M_PI / 180) * SWHEEL_RADIUS;
    
    //Update previous values to be used next loop (DEGREES)
    LPrevPos = LPos;
    RPrevPos = RPos;
    SPrevPos = SPos;

    //Total change in each of the L and R encoders since last reset (INCHES)
    //These are used to calculate the absolute orientation of the bot
    totalDeltaDistL += deltaDistL;
    totalDeltaDistR += deltaDistR;

    //Calculate the current absolute orientation (RADIANS)
    //currentAbsoluteOrientation = THETA_START - ( (totalDeltaDistL - totalDeltaDistR) / (LTrackRadius + RTrackRadius) );
    currentAbsoluteOrientation = (360 - get_rotation()) * M_PI / 180.0;
    currentAbsoluteOrientation = THETA_START + currentAbsoluteOrientation;

    //Calculate the change in the angle of the bot (RADIANS)
    deltaTheta = currentAbsoluteOrientation - previousTheta;

    //Update the previous Theta value (RADIANS)  
    previousTheta = currentAbsoluteOrientation;
    /*
    //If we didn't turn, then we only translated
    if(deltaTheta == 0) {
      deltaXLocal = deltaDistS;
      // could be either L or R, since if deltaTheta == 0 we assume they're =
      deltaYLocal = deltaDistL;
    }
    //Else, caluclate the new local position
    else {
      //Calculate the changes in the X and Y values (INCHES)
      //General equation is:
        //Distance = 2 * Radius * sin(deltaTheta / 2)
      deltaXLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistS / deltaTheta) + STrackRadius);
      deltaYLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistR / deltaTheta) - RTrackRadius);

    }
    */
    //double angular_displacement = (deltaDistR - deltaDistL) / (LTrackRadius);
    double dist = (deltaDistL+deltaDistR)/2;

    // Update the pose of the robot using the inertial heading as the heading measurement
    //deltaXLocal = dist * std::cos(get_rotation() + angular_displacement / 2);
    //deltaYLocal = dist * std::sin(get_rotation() + angular_displacement / 2);
    
    deltaXLocal = dist * std::cos(currentAbsoluteOrientation);
    deltaYLocal = dist * std::sin(currentAbsoluteOrientation);
    
    deltaXGlobal = deltaXLocal;
    deltaYGlobal = deltaYLocal;
    /*
    //The average angle of the robot during it's arc (RADIANS)
    avgThetaForArc = currentAbsoluteOrientation - (deltaTheta / 2);

    deltaXGlobal = (deltaYLocal * cos(avgThetaForArc)) - (deltaXLocal * sin(avgThetaForArc));
    deltaYGlobal = (deltaYLocal * sin(avgThetaForArc)) + (deltaXLocal * cos(avgThetaForArc));
    */
    //Wraps angles back around if they ever go under 0 or over 2 pi
    while(currentAbsoluteOrientation >= 2 * M_PI) {
      currentAbsoluteOrientation -= 2 * M_PI;
    }
    
    while(currentAbsoluteOrientation < 0) {
      currentAbsoluteOrientation += 2 * M_PI;
    }

    //Update global positions
    xPosGlobal += deltaXGlobal;
    yPosGlobal += deltaYGlobal;


    // Brain.Screen.setCursor(4,8);
    // Brain.Screen.print(deltaDistS/deltaTheta);
    // Brain.Screen.setCursor(5,8);
    // Brain.Screen.print((deltaDistL+deltaDistR)/2);
    // Brain.Screen.setCursor(6,8);
    // Brain.Screen.print(deltaDistS);
    // Brain.Screen.setCursor(7,8);
    // Brain.Screen.print(deltaTheta);


    //loop every 20 milliseconds
    task::sleep(10);

  }
  return 1;
}