package org.firstinspires.ftc.teamcode.lib.movement;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.hardware.base.Robot;

import static org.firstinspires.ftc.teamcode.lib.util.GlobalVars.strafeConstant;

public class MyPosition {

  public static double circumfrenceOfWheel = 15.71;
  public static double ticksPerRev = 1440;
  public static double cmPerTick = circumfrenceOfWheel/ticksPerRev;

  public static Robot myRobot;
  public static double moveScalingFactor = 12.56064392;
  public static double turnScalingFactor = 35.694;
  public static double auxScalingFactor = 12.48;//12.6148;
  public static double auxPredictionScalingFactor = -30.0333333333;


  public static double wheelLeftLast = 0.0;
  public static double wheelRightLast = 0.0;
  public static double wheelAuxLast = 0.0;

  public static double lastAngle = 0;

  public static double worldXPosition = 0.0;
  public static double worldYPosition = 0.0;
  public static double worldAngle_rad = 0.0;

  public static double currPos_l = 0;
  public static double currPos_r = 0;
  public static double currPos_a = 0;



  //stuff for reading the angle in an absolute manner
  public static double wheelLeftInitialReading = 0.0;
  public static double wheelRightInitialReading = 0.0;
  public static double lastResetAngle = 0.0;//this is set when you reset the position


  //use this to get how far we have traveled in the y dimension this update
  public static double currentTravelYDistance = 0.0;


  public static void initialize(double l, double r,double a, Robot myRobot){
    MyPosition.myRobot = myRobot;
    currPos_l = l;
    currPos_r = r;
    currPos_a = a;
    update();
  }

  public static void giveMePositions(double l, double r, double a){
    currPos_l = l;
    currPos_r = r;
    currPos_a = a;
    update();
  }

 private static void update(){
    PositioningCalculations();
 }



  /**
   * Makes sure an angle is in the range of -180 to 180
   * @param angle
   * @return
   */
  public static double AngleWrap(double angle){
    while (angle<-Math.PI){
      angle += 2.0*Math.PI;
    }
    while (angle>Math.PI){
      angle -= 2.0*Math.PI;
    }
    return angle;
  }

  public static void PosCalcNewMe(double r, double a){



  }

  public static double[] getRadians(double r, double a){

    return new double[]{r/1440 * 2* Math.PI, a/1440 * 2 * Math.PI};

  }


  //dX = e0 - (dTheta * y0), dY = e1 - (dTheta * x0), and dTheta = imu_delta
  public static void PosCalcNiceArnav(double r, double a){

    double currentAngle = worldAngle_rad;

    currPos_r = r;
    currPos_a = -a;

    double thetaDelta = currentAngle - lastAngle;


    double wheelRightCurrent = currPos_r;
    double wheelAuxCurrent = currPos_a;

    double rightCM = wheelRightCurrent * cmPerTick;
    double auxCM = wheelAuxCurrent * cmPerTick;

    double wheelRightDelta = wheelRightCurrent - wheelRightLast;
    double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;

    double rightDeltaCM = wheelRightDelta * cmPerTick;
    double auxDeltaCM = wheelAuxDelta * cmPerTick;

    //this doenst make sense my guy arnav
    //double deltaX = auxDeltaCM - (thetaDelta * 7.5);
    double deltaX = auxDeltaCM - (thetaDelta * strafeConstant);
    double deltaY = rightDeltaCM;

    worldXPosition += deltaX;
    worldYPosition += deltaY;


    lastAngle = worldAngle_rad;

    //save the last positions for later
    wheelRightLast = wheelRightCurrent;
    wheelAuxLast = wheelAuxCurrent;

  }

  public static void PosCalcNiceWebsite(double r, double a){

    double lastAngle = 0;
    double currentAngle = worldAngle_rad;

    double dTheta = currentAngle - lastAngle;

    double rCm = r * cmPerTick;

    double dX = rCm*Math.cos(currentAngle + (dTheta/2));
    double dY = rCm*Math.sin(currentAngle + (dTheta/2));

    worldXPosition = dX;
    worldYPosition = dY;


    lastAngle = worldAngle_rad;

  }



    public static void PosCalc2Wheel(double r, double a){
        currPos_r = r;
        currPos_a = -a;

        double wheelRightCurrent = currPos_r;
        double wheelAuxCurrent = currPos_a;

        double rightCM = wheelRightCurrent * cmPerTick;
        double auxCM = wheelAuxCurrent * cmPerTick;

        double wheelRightDelta = wheelRightCurrent - wheelRightLast;
        double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;

        double rightDeltaCM = wheelRightDelta * cmPerTick;
        double auxDeltaCM = wheelAuxDelta * cmPerTick;

        //worldXPosition += ((leftDeltaCM + rightDeltaCM)/2);
        //worldYPosition += auxDeltaCM;

        worldYPosition = rightCM;
        worldXPosition = auxCM;

        //save the last positions for later
        wheelRightLast = wheelRightCurrent;
        wheelAuxLast = wheelAuxCurrent;

    }


    public static void PosCalc3Wheel(double l, double r, double a){
      currPos_l = l;
      currPos_r = r;
      currPos_a = a;

      double wheelLeftCurrent = -currPos_l;
      double wheelRightCurrent = currPos_r;
      double wheelAuxCurrent = currPos_a;

      double leftCM = wheelLeftCurrent * cmPerTick;
      double rightCM = wheelRightCurrent * cmPerTick;
      double auxCM = wheelAuxCurrent * cmPerTick;

      double wheelLeftDelta = wheelLeftCurrent - wheelLeftLast;
      double wheelRightDelta = wheelRightCurrent - wheelRightLast;
      double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;

      double leftDeltaCM = wheelLeftDelta * cmPerTick;
      double rightDeltaCM = wheelRightDelta * cmPerTick;
      double auxDeltaCM = wheelAuxDelta * cmPerTick;

      //worldXPosition += ((leftDeltaCM + rightDeltaCM)/2);
      //worldYPosition += auxDeltaCM;

      worldXPosition = ((leftCM+rightCM)/2);
      worldYPosition = auxCM;

      //save the last positions for later
      wheelLeftLast = wheelLeftCurrent;
      wheelRightLast = wheelRightCurrent;
      wheelAuxLast = wheelAuxCurrent;

  }

  /**
   * Updates our position on the field using the change from the encoders
   */
  public static void PosCalcNewPeter(double r, double a){
    double lastAngle = 0;
    double currentAngle = worldAngle_rad;

    double wheelRightCurrent= r;
    double wheelAuxCurrent = a;

    //compute how much the wheel data has changed
    double wheelRightDelta = wheelRightCurrent - wheelRightLast;
    double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;


    //get the real distance traveled using the movement scaling factors
    double wheelRightDeltaScale = wheelRightDelta*cmPerTick;
    double wheelAuxDeltaScale = wheelAuxDelta*cmPerTick;

    double rCM = wheelRightCurrent * cmPerTick;
    double aCM = wheelAuxCurrent * cmPerTick;

    //get how much our angle has changed
    double angleIncrement = currentAngle - lastAngle;
    //myRobot.telemetry.addLine("Angle increment is " +
    //     (angleIncrement > 0 ? "POSITIVE" : "NEGATIVE"));


    //but use absolute for our actual angle
    double wheelRightTotal = r-wheelRightInitialReading;

    //get the predicted amount the strafe will go
    double tracker_a_prediction = Math.toDegrees(angleIncrement)*(auxPredictionScalingFactor);
    //now subtract that from the actual
    double r_xDistance = wheelAuxDeltaScale-tracker_a_prediction;


    //relativeY will by defa
    double relativeY = wheelRightDeltaScale;
    double relativeX = r_xDistance;

    //  myRobot.telemetry.addLine("left wheel: " + (wheelLeftCurrent*moveScalingFactor/1000.0));
    //   myRobot.telemetry.addLine("right wheel: " + (wheelRightCurrent*moveScalingFactor/1000.0));
    //  myRobot.telemetry.addLine("aux wheel: " + (wheelAuxCurrent*auxScalingFactor/1000.0));


    //if angleIncrement is > 0 we can use steven's dumb stupid and stupid well you know the point
    //equations because he is dumb
    if(Math.abs(angleIncrement) > 0){
      //gets the radius of the turn we are in
      double radiusOfMovement = (wheelRightDeltaScale)/(angleIncrement);
      //get the radius of our strafing circle
      double radiusOfStrafe = r_xDistance/angleIncrement;





      relativeY = (radiusOfMovement * Math.sin(angleIncrement)) - (radiusOfStrafe * (1 - Math.cos(angleIncrement)));

      relativeX = radiusOfMovement * (1 - Math.cos(angleIncrement)) + (radiusOfStrafe * Math.sin(angleIncrement));

      //  myRobot.telemetry.addLine("radius of movement: " + radiusOfMovement);
//            myRobot.telemetry.addLine("radius of straif: " + radiusOfStraif);
      //     myRobot.telemetry.addLine("relative y: " + relativeY);
      //     myRobot.telemetry.addLine("relative x: " + relativeX);
    }



    worldXPosition = (Math.cos(worldAngle_rad) * rCM) + (Math.sin(worldAngle_rad) *
        aCM);
    worldYPosition = (Math.sin(worldAngle_rad) * rCM) - (Math.cos(worldAngle_rad) *
        aCM);



    //  SpeedOmeter.yDistTraveled += relativeY;
    // SpeedOmeter.xDistTraveled += r_xDistance;



    //save the last positions for later
    wheelRightLast = wheelRightCurrent;
    wheelAuxLast = wheelAuxCurrent;


    //save how far we traveled in the y dimension this update for anyone that needs it
    //currently the absolute control of the collector radius uses it to compensate for
    //robot movement
    currentTravelYDistance = relativeY;

    lastAngle = worldAngle_rad;
  }


  /**
   * Updates our position on the field using the change from the encoders
   */
  public static void PositioningCalculations(){
    double wheelLeftCurrent = currPos_l;
    double wheelRightCurrent= currPos_r;
    double wheelAuxCurrent = currPos_a;

    //compute how much the wheel data has changed
    double wheelLeftDelta = wheelLeftCurrent - wheelLeftLast;
    double wheelRightDelta = wheelRightCurrent - wheelRightLast;
    double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;


    //get the real distance traveled using the movement scaling factors
    double wheelLeftDeltaScale = wheelLeftDelta*moveScalingFactor/1000.0;
    double wheelRightDeltaScale = wheelRightDelta*moveScalingFactor/1000.0;
    double wheelAuxDeltaScale = wheelAuxDelta*auxScalingFactor/1000.00;

    //get how much our angle has changed
    double angleIncrement = (wheelLeftDelta-wheelRightDelta)*turnScalingFactor/100000.0;
    //myRobot.telemetry.addLine("Angle increment is " +
   //     (angleIncrement > 0 ? "POSITIVE" : "NEGATIVE"));


    //but use absolute for our actual angle
    double wheelRightTotal = currPos_r-wheelRightInitialReading;
    double wheelLeftTotal = -(currPos_l-wheelLeftInitialReading);
    worldAngle_rad = AngleWrap(((wheelLeftTotal-wheelRightTotal)*turnScalingFactor/100000.0) + lastResetAngle);

    //get the predicted amount the straif will go
    double tracker_a_prediction = Math.toDegrees(angleIncrement)*(auxPredictionScalingFactor/10.0);
    //now subtract that from the actual
    double r_xDistance = wheelAuxDeltaScale-tracker_a_prediction;


    //relativeY will by defa
    double relativeY = (wheelLeftDeltaScale + wheelRightDeltaScale)/2.0;
    double relativeX = r_xDistance;

  //  myRobot.telemetry.addLine("left wheel: " + (wheelLeftCurrent*moveScalingFactor/1000.0));
 //   myRobot.telemetry.addLine("right wheel: " + (wheelRightCurrent*moveScalingFactor/1000.0));
  //  myRobot.telemetry.addLine("aux wheel: " + (wheelAuxCurrent*auxScalingFactor/1000.0));


    //if angleIncrement is > 0 we can use steven's dumb stupid and stupid well you know the point
    //equations because he is dumb
    if(Math.abs(angleIncrement) > 0){
      //gets the radius of the turn we are in
      double radiusOfMovement = (wheelRightDeltaScale+wheelLeftDeltaScale)/(2*angleIncrement);
      //get the radius of our straifing circle
      double radiusOfStraif = r_xDistance/angleIncrement;





      relativeY = (radiusOfMovement * Math.sin(angleIncrement)) - (radiusOfStraif * (1 - Math.cos(angleIncrement)));

      relativeX = radiusOfMovement * (1 - Math.cos(angleIncrement)) + (radiusOfStraif * Math.sin(angleIncrement));

    //  myRobot.telemetry.addLine("radius of movement: " + radiusOfMovement);
//            myRobot.telemetry.addLine("radius of straif: " + radiusOfStraif);
 //     myRobot.telemetry.addLine("relative y: " + relativeY);
 //     myRobot.telemetry.addLine("relative x: " + relativeX);
    }



    worldXPosition += (Math.cos(worldAngle_rad) * relativeY) + (Math.sin(worldAngle_rad) *
        relativeX);
    worldYPosition += (Math.sin(worldAngle_rad) * relativeY) - (Math.cos(worldAngle_rad) *
        relativeX);



  //  SpeedOmeter.yDistTraveled += relativeY;
   // SpeedOmeter.xDistTraveled += r_xDistance;



    //save the last positions for later
    wheelLeftLast = wheelLeftCurrent;
    wheelRightLast = wheelRightCurrent;
    wheelAuxLast = wheelAuxCurrent;


    //save how far we traveled in the y dimension this update for anyone that needs it
    //currently the absolute control of the collector radius uses it to compensate for
    //robot movement
    currentTravelYDistance = relativeY;
  }

  /**
   * Updates our position on the field using the change from the encoders
   */
  public static void PositioningCalculationsOld(){
    double wheelLeftCurrent = currPos_l;
    double wheelRightCurrent= -currPos_r;
    double wheelAuxCurrent = currPos_a;

    //compute how much the wheel data has changed
    double wheelLeftDelta = wheelLeftCurrent - wheelLeftLast;
    double wheelRightDelta = wheelRightCurrent - wheelRightLast;
    double wheelAuxDelta = wheelAuxCurrent - wheelAuxLast;


    //get the real distance traveled using the movement scaling factors
    double wheelLeftDeltaScale = wheelLeftDelta*moveScalingFactor/1000.0;
    double wheelRightDeltaScale = wheelRightDelta*moveScalingFactor/1000.0;
    double wheelAuxDeltaScale = wheelAuxDelta*auxScalingFactor/1000.00;



    //get how much our angle has changed
    double angleIncrement = (wheelLeftDelta-wheelRightDelta)*turnScalingFactor/100000.0;



    //but use absolute for our actual angle
    double wheelRightTotal = currPos_r-wheelRightInitialReading;
    double wheelLeftTotal = -(currPos_l-wheelLeftInitialReading);
    worldAngle_rad = AngleWrap(((wheelLeftTotal-wheelRightTotal)*turnScalingFactor/100000.0) + lastResetAngle);




    //relative y translation
    double r_yDistance = (wheelRightDeltaScale+wheelLeftDeltaScale)/2;


    double tracker_a_prediction = Math.toDegrees(angleIncrement)*(auxPredictionScalingFactor/10.0);
    double r_xDistance = wheelAuxDeltaScale-tracker_a_prediction;


    worldXPosition += (Math.cos(worldAngle_rad) * r_yDistance) + (Math.sin(worldAngle_rad) *
        r_xDistance);
    worldYPosition += (Math.sin(worldAngle_rad) * r_yDistance) - (Math.cos(worldAngle_rad) *
        r_xDistance);



//    SpeedOmeter.yDistTraveled += r_yDistance;
  //  SpeedOmeter.xDistTraveled += r_xDistance;



    //save the last positions for later
    wheelLeftLast = wheelLeftCurrent;
    wheelRightLast = wheelRightCurrent;
    wheelAuxLast = wheelAuxCurrent;


    //save how far we traveled in the y dimension this update for anyone that needs it
    //currently the absolute control of the collector radius uses it to compensate for
    //robot movement
    currentTravelYDistance = r_yDistance;

  }
  public static double subtractAngles(double angle1, double angle2){
    return AngleWrap(angle1-angle2);
  }




  /**USE THIS TO SET OUR POSITION**/
  public static void setPosition(double x,double y,double angle){
    worldXPosition = x;
    worldYPosition = y;
    worldAngle_rad= angle;

    //remember where we were at the time of the reset
    wheelLeftInitialReading = currPos_l;
    wheelRightInitialReading = currPos_r;
    lastResetAngle = angle;
  }

  ////////////////////////////////////////////////////////////////////////////////


  public static float AngleWrap(float angle){
    while (angle<-Math.PI){
      angle += 2*Math.PI;
    }
    while (angle>Math.PI){
      angle -= 2*Math.PI;
    }
    return angle;
  }
}
