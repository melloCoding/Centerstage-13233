package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.logging.Level;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.lang.annotation.Target;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.io.FileOutputStream;
import java.io.File;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import java.util.Set;
import java.util.Map;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// import for IMU (gyroscope)
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

// imports for TensorFlow
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;



@Autonomous(name = "TwoPixelAuto", group = "")

public class TwoPixelAuto extends LinearOpMode {
   
    private DcMotor FLMoto;
    private DcMotor FRMoto;
    private DcMotor BLMoto;
    private DcMotor BRMoto;
    
    //
    // Hardware for pixel intake
    //
    private DcMotor intakeArm;
    private DcMotor intakeArmRaise;
    private CRServo intakeServoLeft;
    private CRServo intakeServoRight;
    private Servo intakeRotateServo;
    
    private IMU imu;
    private Orientation lastAngle = new Orientation();
    private VoltageSensor VoltSens;
    private static double powerConstant = .5;
    
    private DistanceSensor distSensor;
    //Drone
    private Servo droneLaunch;
    
    //BlinkinLEDs
    private RevBlinkinLedDriver blinkinLedDriver;
        
    private RevBlinkinLedDriver.BlinkinPattern BasePattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    private RevBlinkinLedDriver.BlinkinPattern StopPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    private RevBlinkinLedDriver.BlinkinPattern Park1Pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    private RevBlinkinLedDriver.BlinkinPattern Park2Pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    private RevBlinkinLedDriver.BlinkinPattern Park3Pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
    
    /*********************/
    /* Variables for imu */
    /*********************/
    double            globalAngle, correction;
    YawPitchRollAngles   lastAngles;
    
    // Use Voltage reading to try to maintain consistent power during Auto
    double voltage = 0;
    double batteryConst = 13.5;
    double powerConst;
    
    /***************************/
    /* Possible Automous Modes */
    /***************************/
    public enum AutoMode
    {
       AUTO_MODE_NOT_SELECTED,
       AUTO_MODE_BLUEBACK,
       AUTO_MODE_REDBACK,
       AUTO_MODE_BLUEFRONT,
       AUTO_MODE_REDFRONT,
     }

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
 
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/13233CustModCenterStage.tflite";
    
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
       // "Pixel",
       "BlueGE",
       "RedGE"
    };
 
    public enum GELocation
    {
       LOC_LOCATIONRIGHT,
       LOC_LOCATIONCENTER,
       LOC_LOCATIONLEFT,
       LOC_NOT_SELECTED
    }
              
    private static final Double wheelCircumference = 4.0 * Math.PI;
    private static final Double gearRatio = 1.0;
    private static final Double countsPerRotation = 2240.0;
    private static final Double scaleFactor = 0.3;
    private static final Double countsPerInch =  countsPerRotation / wheelCircumference / gearRatio * scaleFactor;

    
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
        

    // Gobal Variables
    private static Double noPower = 0.0;
    private static Double quarterPower = 0.25;
    private static Double oneThirdPower = 0.34;
    private static Double halfPower = 0.5;
    private static Double threeQuartPower = 0.65;
    private static Double fullPower = 1.0;
    
    // Pixel Arm Positions Variables (need to find actual values during testing)
    private static int posBottom = 0;
    private static int posForMoving = -200;
    private static int posBackdrop = -2800;
    
    // Sleep Variables
    private static int sleepTimeNone = 0;

    
    // Global Variables to store Game Specific Information
    private GELocation  GELocationDetected = GELocation.LOC_NOT_SELECTED;     // location to place purple pixel
    AutoMode autoMode = AutoMode.AUTO_MODE_NOT_SELECTED;                      // automous mode selected
  
    
    /*******************************************************************************************/
    /* Function: SelectAutoMode                                                                */
    /* Returns: Selected mode                                                                  */
    /*                                                                                         */
    /* This function is use to select the automous code to be executed for this match          */
    /* Game pad 1 is used and the following buttons are used for selection:                    */
    /*      a - Red alliance Backstage                                                         */
    /*      b - Blue alliance Backstage                                                        */
    /*      x - Red alliance Frontstage                                                        */
    /*      y - Blue alliance Frontstage                                                       */
    /*******************************************************************************************/
    private AutoMode SelectAutoMode() {
 
       AutoMode autoMode = AutoMode.AUTO_MODE_NOT_SELECTED;   // Local variable to store selected automous mode
 
       /*******************************************/
       /* Display automous mode not selected yet  */
       /*******************************************/
       telemetry.addData("AutoMode","Not Selected");
       telemetry.addData("AutoMode","Select a - Red alliance in the backstage");
       telemetry.addData("AutoMode","Select b - Blue alliance in the backstage");
       telemetry.addData("AutoMode","Select x - Red alliance in the frontstage");       
       telemetry.addData("AutoMode","Select y - Blue alliance in the frontstage");

       telemetry.update();
 
       /*****************************************/
       /* Loop until automous mode is selected  */
       /*****************************************/
       while (!isStopRequested() && autoMode == AutoMode.AUTO_MODE_NOT_SELECTED)  {
          
          if (gamepad1.a) {  // Red side away from audience/Blue side toward audience
             autoMode = AutoMode.AUTO_MODE_REDBACK;
          }
        
          if (gamepad1.b) {  // Red side toward audience/Blue side away from audience         
             autoMode = AutoMode.AUTO_MODE_BLUEBACK;
          }
          
          if (gamepad1.y) {
              autoMode = AutoMode.AUTO_MODE_BLUEFRONT;
          }
          
          if (gamepad1.x){
              autoMode = AutoMode.AUTO_MODE_REDFRONT;
          }
  
         idle();
       }

       /************************************/
       /* Display selected automous mode   */
       /************************************/
       // mode.setValue(autoMode.toString());
       telemetry.addData("Automous Mode", autoMode.toString());
       telemetry.update();

      // Wait for the user to release the button
      while (!isStopRequested() && (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y)) {
         idle();
      } 
 
      return autoMode;
   }


    /*******************************************************************************************/
    /* Function: SelectDelayTime                                                               */
    /* Returns: Delay Time in milliseconds                                                     */
    /*                                                                                         */
    /* This function is use to select how long to delay the start of the automous code.        */
    /* Game pad 1 is used and the following controls are used for selection:                   */
    /*      left bumper - decrease delay time by 1000 milliseconds (1 second)                  */
    /*      right bumper - increase delay time by 1000 milliseconds (1 second)                 */
    /*      a button -  set selected time                                                      */
    /*                                                                                         */
    /*                  note: if no delay time is needed, just select the a button.  The       */
    /*                        default for the delay time is 0.                                 */
    /*******************************************************************************************/
    private Integer SelectDelayTime() {
      Integer delayTimeMilliseconds = 0;    // Initialize delay to be 0 seconds

      // display delay time not set 
      telemetry.addData("Delay","%d (Not Set)",delayTimeMilliseconds);
      telemetry.addData("Delay Time","left bumper - decrease delay time by (1 second)");
      telemetry.addData("Delay Time","right bumper - increase delay time by (1 second)");
      telemetry.addData("Delay Time","no delay time is needed, just select the a button");
      telemetry.update();

      /*******************************************************************************/
      /* Select Delay time.                                                          */
      /*   - Select 'a' button without hitting bumpers if no delay needed            */
      /*   - Use Left Bumper to decrease delay time                                  */
      /*   - Use Right bumper to increase delay time                                 */
      /*                                                                             */
      /* Note:  After entering delay time, use "a" button to set selected time       */
      /*******************************************************************************/
      while (!isStopRequested() && gamepad1.a == false)  {
         
         if (gamepad1.left_bumper) {
            delayTimeMilliseconds -= 1000;
            
            // ensure delay time does not go negetive
            if (delayTimeMilliseconds < 0) {
               delayTimeMilliseconds = 0;
            }
         
            // Wait for the bumper to be released
            while (gamepad1.left_bumper) {
               idle();
            }
            
            telemetry.addData("Delay","%d (decrease)",delayTimeMilliseconds);
            telemetry.update();
         }
         
         if (gamepad1.right_bumper) {
            delayTimeMilliseconds += 1000;
            
            // ensure delay time is not greater than 20 seconds
            if (delayTimeMilliseconds > 20000) {
                delayTimeMilliseconds = 20000;
            }
            
            while (gamepad1.right_bumper) {
               idle();
            }
            telemetry.addData("Delay","%d (increase)",delayTimeMilliseconds);
            telemetry.update();
         }
         
      }

      // Wait for user to release the a button
      while (!isStopRequested() && gamepad1.a) {
         idle();
      }
     
      /**************************************/
      /* Display selected delay time        */
      /**************************************/
      // delay.setValue("%d", delayTimeMilliseconds);
      telemetry.addData("Delay Time SET", delayTimeMilliseconds);
      telemetry.update();
      return delayTimeMilliseconds;    // returns selected delay time
   } 

   /******************************/
   /* OpMode for automous code   */
   /******************************/
   @Override
   public void runOpMode() throws InterruptedException {
        
       double tgtPower = 0;
        
       telemetry.setAutoClear(false);
       telemetry.addData("Status", "Initializing");
       telemetry.update();
       
       /*********************
          Map All Motors   
        ********************/

       //****************************************//
       // Map Drive Train motors                 //
       //    - BR means "back right"             //
       //    - BL means "back left"              //
       //    - FR means "front right"            //
       //    - FL means "front left"             //
       //****************************************//
        FLMoto = hardwareMap.dcMotor.get("FLMoto");
        FRMoto = hardwareMap.dcMotor.get("BRMoto");
        BLMoto = hardwareMap.dcMotor.get("BLMoto");
        BRMoto = hardwareMap.dcMotor.get("FRMoto");
        
        //*******************************//
        // Motors for delivering pixels  //
        //*******************************//
        intakeArm = hardwareMap.dcMotor.get("intakeArm");
        intakeArmRaise = hardwareMap.dcMotor.get("intakeArmRaise");

      
        /********************/
        /* Map All Servos   */
        /********************/

        //*******************************//
        // Servos for delivering pixels  //
        //*******************************//
        intakeArm = hardwareMap.dcMotor.get("intakeArm");
        intakeServoLeft = hardwareMap.crservo.get("intakeServoLeft");
        intakeServoRight = hardwareMap.crservo.get("intakeServoRight");
        intakeRotateServo = hardwareMap.servo.get("intakeRotateServo");
  
        //Dronelaunch servo
        droneLaunch = hardwareMap.servo.get("droneLaunch");
        
        /********************/
        /* Map All Sensors  */
        /********************/     

        //distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");  // not used yet
       
        VoltSens = hardwareMap.voltageSensor.get("Control Hub");
       
       
       /************************/
       /* Setup IMU parameters */
       /************************/
       
       // Retrieve and initialize the IMU. The IMU should be attached to 
       // IC2 port 0 on a Core Device Interface Module
       imu = hardwareMap.get(IMU.class, "imu");
       
       /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
       */
       RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
       RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

       telemetry.addData("Mode","calibrating imu...." );
       telemetry.update();
       
       try {
           RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
           imu.initialize(new IMU.Parameters(orientationOnRobot));
            
           telemetry.addData("imu calib status", "calibrated" );
           telemetry.update();
            
           // initialize imu global variables after calibrating imu
           resetAngle();
            
       } catch (IllegalArgumentException e) {
           telemetry.addData("imu calib status", "failed - try again");
           telemetry.update();
       }
      
            
       // Put initialization blocks here.
       
       // Initalize camera
       initTfod();
       
       // set direction of motors
        FLMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        FRMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        BLMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        
                
       // Set the intakeArmRaise motor (moves the claw arm up and down)
       // direction and zero power behavior.  Also set target position to 0.
       intakeArmRaise.setDirection(DcMotorSimple.Direction.REVERSE);
       intakeArmRaise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       intakeArmRaise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       intakeArmRaise.setTargetPosition(posBottom);
        
        //Distance Sensor (Not used yet)

        // Variables for intake arm
        int armMotoPosition = 0;
        int intakeArmRaisePosition = 0;

        // Put initialization for Intake Arm hardware 
        // intakeRotateServo need to be down so the claw is 
        // not in the way.
        
        //intakeArmRaise moves the claw arm up and down
        //intakeArmRaise.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeArmRaise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmRaise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmRaise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        intakeArmRaise.setTargetPosition(0);
        droneLaunch.setPosition(0.4);
    
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        
        //BlinkinLEDs
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
        blinkinLedDriver.setPattern(BasePattern);
        
        // telemetry.addData("Pattern: ", BasePattern.toString());
        // telemetry.update();
       
       // Create local variable to store amount of delay time
       Integer delayTimeMilliseconds = 0;     // variable to store how long to delay before starting automous
        
       /***********************************************************/
       /* Select Automous Mode and Delay time                     */
       /***********************************************************/
       autoMode = SelectAutoMode();                   
       delayTimeMilliseconds = SelectDelayTime();     

       /**********************************************************/ 
       /* All required data entered.  Automous is initialized.   */
       /**********************************************************/
        telemetry.addData("Status", "Initialized");
        telemetry.addData("mode","waiting for start");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        
        
       /**********************************/
       /* Wait for start of the match    */
       /**********************************/
       waitForStart();
       
       telemetry.clearAll();               // clear display messages

       telemetry.addData("Mode", "running");
       telemetry.update();

       resetEncoders();

       // Call function to use TensorFlow to find Game Element location
       findGELocation();
       
       // Delay start if needed
       if (delayTimeMilliseconds > 0) {
          telemetry.addData("Status","Delaying");    // display delay status
          telemetry.update();
          
          sleep(delayTimeMilliseconds);                       // wait selected amount of time              
          
          telemetry.addData("Status","Running");  // dispay delay over and automous code is running
          telemetry.update();

       }

       //
       // Common code for all locations to get robot between spikes
       //
       
       // rotate claw up so it doesn't drag on the floor
       intakeRotateServo.setPosition(0.0);
       driveForward(8.0, halfPower);
       
       // Determind which automous code to run
       switch (autoMode) {
          case AUTO_MODE_REDBACK:
              RedBackStage();
              break;
 
          case AUTO_MODE_BLUEBACK:
              BlueBackStage();
              break;
          
          case AUTO_MODE_REDFRONT:
              RedFrontStage();
              break;
              
          case AUTO_MODE_BLUEFRONT:
              BlueFrontStage();
              break;
 
          case AUTO_MODE_NOT_SELECTED:
             // This one should not happen if it does do nothing
             break;
       } 
    }
    
    /***************************************************************************/
    /* Function: BlueBackstage                                                 */
    /* Returns: none                                                           */
    /* Uses Global Variables: GELocationDetected                               */
    /*                                                                         */
    /* This function is for when the robot is in the blue alliance and starts  */
    /* in the backstage location                                               */
    /*                                                                         */
    /* The robot will place the preloaded purple pixel on the spike mark that  */
    /* contains game element.  The robot will then move to the backstage area  */
    /* where it will place the yellow pixel on the backdrop and then park next */
    /* to the wall so the other team can have access to the backdrop and then  */
    /* park.                                                                   */
    /***************************************************************************/
    private void BlueBackStage() {
       
       switch (GELocationDetected) {
          case LOC_LOCATIONRIGHT:
              // rotate to the right to face right spike
              rotate(90, halfPower);
              releasePurplePixel();
              driveBack(5.0,quarterPower);
              
              // turn the robot around to face the backboard
              rotate(180, halfPower);
                           
              // move towards the backboard
              driveForward(36.0, halfPower);
              
              // place pixel on backboard
              placePixelBB();
              
              // move to the left to park              
              strafeLeft(13.0, halfPower);
              
              break;
 
          case LOC_LOCATIONCENTER:
              // drive forward to get closer to the line if needed.
              driveForward(4.0, halfPower);
              releasePurplePixel();
              
              // move back so the robot doesn't hit the pixel
              driveBack(4.0,quarterPower);
              
              // rotate to the left to face the backboard
              rotate(-90, halfPower);
              
              // move towards the backboard to place the pixel
              driveForward(36.0, halfPower);
              
              // place pixel on backboard
              placePixelBB();
              
              // move to the right to park              
              strafeLeft(13.0, halfPower);
              
              break;
          
          case LOC_LOCATIONLEFT:
              // rotate to the left to face left spike
              rotate(-90, halfPower);
              releasePurplePixel();
              
              driveBack(2.0,quarterPower);
              
              // move to the right so the robot dosen't runover the pixel              
              strafeLeft(13.0, halfPower);
              
              // move towards the backboard
              driveForward(36.0, halfPower);
              
              // move back to the left to get in front of the backboard
              // for the correct location for the right spike
              strafeRight(13.0, halfPower);
              
              // place pixel on backboard
              placePixelBB();
              
              // move to the right to park              
              strafeLeft(13.0, halfPower);
              break;

       } 
       
    } // End of BlueBackStage()
    

    /***************************************************************************/
    /* Function: RedBackStage                                                  */
    /* Returns: none                                                           */
    /* Uses Global Variables: GELocationDetected                               */
    /*                                                                         */
    /* This function is for when the robot is in the red alliance and starts   */
    /* in the backstage location                                               */
    /*                                                                         */
    /* The robot will place the preloaded purple pixel on the spike mark that  */
    /* contains game element.  The robot will then move to the backstage area  */
    /* where it will place the yellow pixel on the backdrop and then park next */
    /* to the wall so the other team can have access to the backdrop and then  */
    /* park.                                                                   */
    /***************************************************************************/
    private void RedBackStage() {
       
       // Robot should be in the middle of the spikes (lines)
       // place purple pixel on correct spike
       
       switch (GELocationDetected) {
          case LOC_LOCATIONRIGHT:
              // rotate to the right to face right spike
              rotate(90, halfPower);
              releasePurplePixel();
              driveBack(2.0,quarterPower);
              
              // move to the right so the robot dosen't runover the pixel              
              strafeRight(13.0, halfPower);
              
              // move towards the backboard
              driveForward(36.0, halfPower);
              
              // move back to the left to get in front of the backboard
              // for the correct location for the right spike
              strafeLeft(13.0, halfPower);
              
              // place pixel on backboard
              placePixelBB();
              
              // move to the right to park              
              strafeRight(13.0, halfPower);
              
              break;
 
          case LOC_LOCATIONCENTER:
              // drive forward to get closer to the line if needed.
              driveForward(4.0, halfPower);
              releasePurplePixel();
              
              // move back so the robot doesn't hit the pixel
              driveBack(4.0,quarterPower);
              
              // rotate to the right to face the backboard
              rotate(90, halfPower);
              
              // move towards the backboard to place the pixel
              driveForward(36.0, halfPower);
              
              // place pixel on backboard
              placePixelBB();
              
              // move to the right to park              
              strafeRight(13.0, halfPower);
              
              break;
          
          case LOC_LOCATIONLEFT:
              // rotate to the left to face left spike
              rotate(-90, halfPower);
              releasePurplePixel();
              driveBack(5.0,quarterPower);
              
              // turn the robot around to face the backboard
              rotate(180, halfPower);
                           
              // move towards the backboard
              driveForward(36.0, halfPower);
              
              // place pixel on backboard
              placePixelBB();
              
              // move to the left to park              
              strafeRight(13.0, halfPower);
              break;

       } 
       

    }  // End of RedBackStage()


    /***************************************************************************/
    /* Function: RedFrontStage                                                 */
    /* Returns: none                                                           */
    /* Uses Global Variables: GELocationDetected                               */
    /*                                                                         */
    /* This function is for when the robot is in the red alliance and starts   */
    /* in the front (audience) location                                        */
    /*                                                                         */
    /* The robot will place the preloaded purple pixel on the spike mark that  */
    /* contains game element.  The robot will then move under the stagedoor to */
    /* the backstage area where it will place the yellow pixel on the backdrop */ 
    /* and then park away from the wall.                                       */
    /***************************************************************************/    
    private void RedFrontStage()
    {
       switch (GELocationDetected) {
          case LOC_LOCATIONRIGHT:
              // rotate to the right to face right spike
              rotate(90, halfPower);
              releasePurplePixel();
              driveBack(2.0,quarterPower);
              
              break;
                //center location
          case LOC_LOCATIONCENTER:
              // drive forward to get closer to the line if needed.
              driveForward(4.0, halfPower);
              releasePurplePixel();
              
              // move back so the robot doesn't hit the pixel
              driveBack(4.0,quarterPower);
              
              break;
          
          case LOC_LOCATIONLEFT:
              // rotate to the left to face left spike
              rotate(-90, halfPower);
              releasePurplePixel();
              driveBack(5.0,quarterPower);

              break;
       } 
    } // End of RedFrontStage()
    

    /***************************************************************************/
    /* Function: BlueFrontStage                                                */
    /* Returns: none                                                           */
    /* Uses Global Variables: GELocationDetected                               */
    /*                                                                         */
    /* This function is for when the robot is in the blue alliance and starts   */
    /* in the front (audience) location                                        */
    /*                                                                         */
    /* The robot will place the preloaded purple pixel on the spike mark that  */
    /* contains game element.  The robot will then move under the stagedoor to */
    /* the backstage area where it will place the yellow pixel on the backdrop */ 
    /* and then park away from the wall.                                       */
    /***************************************************************************/  
    private void BlueFrontStage()
    {
       switch (GELocationDetected) {
          case LOC_LOCATIONRIGHT:
              // rotate to the right to face right spike
              rotate(90, halfPower);
              releasePurplePixel();
              driveBack(5.0,quarterPower);
              
              break;
 
          case LOC_LOCATIONCENTER:
              // drive forward to get closer to the line if needed.
              driveForward(4.0, halfPower);
              releasePurplePixel();
              
              // move back so the robot doesn't hit the pixel
              driveBack(4.0,quarterPower);
              
              break;
          
          case LOC_LOCATIONLEFT:
              // rotate to the left to face left spike
              rotate(-90, halfPower);
              releasePurplePixel();
              
              driveBack(2.0,quarterPower);
              
              break;

       } 
       
    } // End of BlueFrontStage()
    
    /************************************************************/
    /* Function: releasePurplePixel                             */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to release the preloaded purple  */
    /* pixel from the right intake.                             */
    /************************************************************/
    private void releasePurplePixel() {
         // lower the claw
         intakeRotateServo.setPosition(0.5);
         
         // spin the right intake server to release the preloaded
         // pixel
         intakeServoRight.setPower(-0.60);
         sleep(500);
         intakeServoRight.setPower(0.0);
         
         // raise the claw
         intakeRotateServo.setPosition(0.0);
    }
    
    /************************************************************/
    /* Function: placePixelBB                                   */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to place the preloaded yellow    */
    /* pixel from the left intake onto the backboard.           */
    /************************************************************/
    private void placePixelBB() {
        // raise arm and put claw in correct pos
        intakeArmRaise.setTargetPosition(220);
        intakeArmRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmRaise.setPower(1);
        intakeRotateServo.setPosition(0.4);
        
        driveForward(4.0,quarterPower);
        
        // spin the left intake server to release the preloaded
        // pixel
        intakeServoLeft.setPower(0.60);
        sleep(500);
        intakeServoLeft.setPower(0.0);
        
        driveBack(4.0,quarterPower);
         
        intakeArmRaise.setTargetPosition(2);
        intakeArmRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmRaise.setPower(1);
        intakeRotateServo.setPosition(0.0);
    }
    
    /************************************************************/
    /* Function: resetEncoders                                  */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to stop and reset the encoders   */
    /* on all 4 drive train motors.                             */
    /************************************************************/
    private void resetEncoders() {
       FRMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       BRMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       FLMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       BLMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /************************************************************/
    /* Function: runUsingEncoders                               */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to set all 4 drive train motors  */
    /* to run using encoders.                                   */
    /************************************************************/
    private void runUsingEncoders() {
       FRMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       BRMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       FLMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       BLMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /************************************************************/
    /* Function: setDrivePower                                  */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to set the power of all 4 drive  */
    /* train motors                                             */
    /************************************************************/
    private void setDrivePower(double FLpower,double FRpower, Double BLpower, Double BRpower) {  
       FRMoto.setPower(FRpower);
       BRMoto.setPower(BRpower);
       FLMoto.setPower(FLpower);
       BLMoto.setPower(BLpower);
    }
    

    /************************************************************/
    /* Function: driveForward                                   */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move forward   */
    /************************************************************/
    private void driveBack(Double inches,Double power)
    {
        driveForwardInch(inches, -power, -power, -power, -power);
    }
    
    /************************************************************/
    /* Function: driveBack                                      */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move in        */
    /* reverse                                                  */
    /************************************************************/    
    private void driveForward(Double inches,Double power)
    {
        driveForwardInch(inches, power, power, power, power);
    }
    
    /************************************************************/
    /* Function: strafeRight                                    */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move sideways  */
    /* in a right direction                                     */
    /************************************************************/
    private void strafeRight(Double inches,Double power)
    {
        driveForwardInch(inches, power, power, -power, -power);
    }

    /************************************************************/
    /* Function: strafeLeft                                     */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move sideways  */
    /* in a left  direction                                     */
    /************************************************************/
    private void strafeLeft(Double inches,Double power)
    {
        driveForwardInch(inches, -power, -power, power, power);
    }
    
    
    /************************************************************/
    /* Function: driveForwardInch                               */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move in a      */
    /* given direction based on the motor values passed in.     */
    /*                                                          */
    /* forward - all motors positive                            */
    /* backwards - all motors negative                          */
    /* strafe right -                                           */
    /* strafe left -                                            */ 
    /*                                                          */
    /************************************************************/ 
    private void driveForwardInch(Double inches,Double FLpower,Double FRpower, Double BLpower, Double BRpower) {
       
       Double counts = inches * countsPerInch;
              
       resetEncoders();
       runUsingEncoders();
       
       voltage = VoltSens.getVoltage();
       double FLpowerCont = ((batteryConst*FLpower)/voltage);
       double FRpowerCont = ((batteryConst*FRpower)/voltage);
       double BLpowerCont = ((batteryConst*BLpower)/voltage);
       double BRpowerCont = ((batteryConst*BRpower)/voltage);
       
       setDrivePower(FLpower, FRpower, BLpower, BRpower);
    
       while (opModeIsActive() && 
          (Math.abs(FLMoto.getCurrentPosition()) + Math.abs(FRMoto.getCurrentPosition()) /2) 
                     < Math.abs(counts)) {       
                         
           // Use gyro to drive in a straight line.
           correction = checkDirection();
           
           // telemetry.addData("1 imu heading", lastAngles.firstAngle);
           // telemetry.addData("2 global heading", globalAngle);
           // telemetry.addData("3 correction", correction);
           // telemetry.update();
           
           setDrivePower(FLpower-correction, FRpower+correction, BLpower-correction, BRpower+correction);
           idle();
           
       }
 
       setDrivePower(noPower,noPower,noPower,noPower);     // Stop all motors

    }
    
    
    /***************************************************/
    /* Resets the cumulative angle tracking to zero.   */
    /***************************************************/
    private void resetAngle()
    {   
        imu.resetYaw();
        lastAngles = imu.getRobotYawPitchRollAngles();

        globalAngle = 0;
    }

    /************************************************************/
    /* Get current cumulative angle rotation from last reset.   */
    /* @return Angle in degrees. + = left, - = right.           */
    /************************************************************/
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        // Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        
        double deltaAngle = orientation.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = orientation;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        setDrivePower(leftPower,rightPower,leftPower,rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}
       
        // turn the motors off.
        setDrivePower(0.0,0.0,0.0,0.0);
        
        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    } 

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            // With the following lines commented out, the default TfodProcessor Builder
            // will load the default model for the season. To define a custom model to load, 
            // choose one of the following:
            //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
            //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            .setModelFileName(TFOD_MODEL_FILE)

            // The following default settings are available to un-comment and edit as needed to 
            // set parameters for custom models.
            .setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera to use webcam(webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.80f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /***************************** findParkLoc ******************************
     * Uses Tensor Flow to determine the location to place pixel
     ************************************************************************/
    private void findGELocation() 
    {
        // blinkinLedDriver.setPattern(BasePattern);
    
        // GE location = LOC_NOT_SELECTED, LOC_LOCATIONRIGHT, LOC_LOCATIONCENTER, LOC_LOCATIONLEFT  
        // set GE location to LEFT in case the GAME ELEMENT is not detected since only part of the 
        // game element is visible on the left side
        GELocationDetected = GELocation.LOC_LOCATIONLEFT;
   
        if (tfod != null) 
        {
           List<Recognition> currentRecognitions = tfod.getRecognitions();

           if (currentRecognitions != null) // something new was found
           {
              // Step through the list of recognitions and display info for each one.
              for (Recognition recognition : currentRecognitions) {
                   double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                   double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                   
                   telemetry.addData(""," ");
                   telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                   telemetry.addData("- Position", "%.0f / %.0f", x, y);
                   telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                   
                   // With testing determing what can be used to set spike location for pixel              
                   if (x <= 350 ) {
                        telemetry.addData("GE Location", " Center");
 
                        GELocationDetected = GELocation.LOC_LOCATIONCENTER;
                        // blinkinLedDriver.setPattern(Park1Pattern);

                        telemetry.update();
                        visionPortal.close(); 
                        
                        return;

                    }
                    else {
                         telemetry.addData("GE Location", " Right");
 
                        GELocationDetected = GELocation.LOC_LOCATIONRIGHT;
                        // blinkinLedDriver.setPattern(Park1Pattern);

                        telemetry.update();
                        visionPortal.close(); 
                        
                        return;
                    }
  
                }

            } else 
            {
                telemetry.addData("Object Detected", "NONE"); 
            }
   
           telemetry.update();
           visionPortal.close();

        } // end if tfod != null
    }   // end findGELocation()
   
} // end class TwoPixelAuto m