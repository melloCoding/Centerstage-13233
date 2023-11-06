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
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import java.io.FileOutputStream;
import java.io.File;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import java.util.Set;
import java.util.Map;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
// import com.qualcomm.hardware.bosch.BNO055IMU;
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

// import for Vuforia
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;
//import com.vuforia.Image;
//import com.vuforia.PIXEL_FORMAT;
//import com.vuforia.Vuforia;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;



@Autonomous(name = "AutoAllWithTF4cones ", group = "")

public class AutoAllWithTF4cones extends LinearOpMode {
   
    private DcMotor FLMoto;
    private DcMotor FRMoto;
    private DcMotor BLMoto;
    private DcMotor BRMoto;
    
    //Liner Slider
    private DcMotor liftMoto;
  
    private Servo RS_Claw;
    private Servo LS_Claw;
    
    private IMU imu;
    private Orientation lastAngle = new Orientation();
    private VoltageSensor VoltSens;
    private static double powerConstant = .5;
    
    private DistanceSensor distSensor;
    

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
    double        globalAngle, correction;
    YawPitchRollAngles   lastAngles;
    
    double voltage = 0;
    double batteryConst = 13.5;
    double powerConst;
    
    /***************************/
    /* Possible Automous Modes */
    /***************************/
    public enum AutoMode
    {
       AUTO_MODE_NOT_SELECTED,
       AUTO_MODE_LEFT,
       AUTO_MODE_RIGHT,
     }

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
        
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/myCustomPowerPlayModel1.tflite";

   /* private static final String[] LABELS = {
      "1 Bolt",
      "2 Bulb",
      "3 Panel"
    }; */
    
     private static final String[] LABELS = {
      "blueSquare",
      "pinkTriangle",
      "yellowCookie",
    };
 
    public enum parkLocation
    {
       LOC_LOCATION1,
       LOC_LOCATION2,
       LOC_LOCATION3,
       LOC_NOT_SELECTED
    }
    
          
    private static final Double wheelCircumference = 4.0 * Math.PI;
    private static final Double gearRatio = 1.0;
    private static final Double countsPerRotation = 2240.0;
    private static final Double scaleFactor = 0.3;
    private static final Double countsPerInch =  countsPerRotation / wheelCircumference / gearRatio * scaleFactor;

    // Add Vuforia Variables
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix phoneLocationOnRobot = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
   // VuforiaLocalizer vuforia;
    
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AXK2+Q//////AAABmXkdsFZ1sUa9jA4uuZQ6jkcHNGGYStEDQ8eM/w/Gye+u5s3NqHAKx1vlGU1pQO9tEekp3Y/hWWEi3fWmfqmgezOOYnXq+9+BVoZSUS+kWzlF0FcDUe42Xvu2+aWtJ7xpjZXY2HY7ciUCVmZ+/paabtXrVtZDDYgtmxgHWoHN6NODL76PV497bREE8sWBLWJdpE+895noqVZ+fYVUhmutLGf1UzdRoo8e3M2s8Prh51wKefpqulfhZdZB34TWQQUKv9Tlk61QYhzBWKMUZdr323LWYqsF6FX3eS/tz9QzCqLUGMlfwWcR5dnDpt8ChcNXqW17R6vu7QVEuMdl2RnK2M9PhXIiuLegedIxB4nUgZpX ";

 
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    
    /**
     * We use units of mm here because that's the recommended units of measurement for the
     * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
     *      <ImageTarget name="stones" size="247 173"/>
     * You don't *have to* use mm here, but the units here and the units used in the XML
     * target configuration files *must* correspond for the math to work out correctly.
    */
    private static float mmPerInch        = 25.4f;
    private static float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    private static float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    private static float mmTargetHeight   = (4) * mmPerInch;           // the height of the center of the target IncompatibleClassChangeError
    private static float stoneZ           = 2.00f * mmPerInch;         // constant for stone target
    
    private static float CAMERA_FORWARD_DISPLACEMENT = 7.0f * mmPerInch;  // camera is 6 inches in front of robot-center
    private static float CAMERA_VERTICAL_DISPLACEMENT = 7.0f * mmPerInch;  // camera is 6 inches above the ground
    private static float CAMERA_LEFT_DISPLACEMENT = 15.0f * mmPerInch;     // camera is 6 inches right of center 
    
    // Gobal Variables
    private static Double noPower = 0.0;
    private static Double quarterPower = 0.25;
    private static Double oneThirdPower = 0.34;
    private static Double halfPower = 0.5;
    private static Double threeQuartPower = 0.65;
    private static Double fullPower = 1.0;
    
    // Terminal Positions
    private static int posBottom = 0;
    private static int posPreload = -200;
    private static int posLowTerm = -2800;
    private static int posMediumTerm = -4500;
    private static int posHighTerm = -5700;
    private static int posFirstCone = -1015;
    private static int posSecondCone = -800;
    
    private static int sleepTimeNone = 0;
    private static int sleepTimePre = 500;
    private static int sleepTimeFirst = 500;
    private static int sleepTimeLow = 500;
    private static int sleepTimeMed = 800;
    private static int sleepTimeHigh = 1000;
    
    private parkLocation parkLocationDetected = parkLocation.LOC_LOCATION2;
    AutoMode autoMode = AutoMode.AUTO_MODE_NOT_SELECTED;      // variable to store automous mode selected
  
    
    /*******************************************************************************************/
    /* Function: SelectAutoMode                                                                */
    /* Returns: Selected mode                                                                  */
    /*                                                                                         */
    /* This function is use to select the automous code to be executed for this match          */
    /* Game pad 1 is used and the following buttons are used for selection:                    */
    /*      a - Right Side - Red side away from audience/Blue side toward audience             */
    /*      b - Left Side - Red side toward audience/Blue side away from audience              */
    /*******************************************************************************************/
    private AutoMode SelectAutoMode() {
 
       AutoMode autoMode = AutoMode.AUTO_MODE_NOT_SELECTED;   // Local variable to store selected automous mode
 
       /*******************************************/
       /* Display automous mode not selected yet  */
       /*******************************************/
       telemetry.addData("AutoMode","Not Selected");
       telemetry.addData("AutoMode","a - RIGHT - Red side away from audience/Blue side toward audience");
       telemetry.addData("AutoMode","b - LEFT - Red side toward audience/Blue side away from audience");
       telemetry.update();
 

       /*****************************************/
       /* Loop until automous mode is selected  */
       /*****************************************/
       while (!isStopRequested() && autoMode == AutoMode.AUTO_MODE_NOT_SELECTED)  {
          if (gamepad1.a) {  // Red side away from audience/Blue side toward audience
             autoMode = AutoMode.AUTO_MODE_RIGHT;
          }
        
          if (gamepad1.b) {  // Red side toward audience/Blue side away from audience         
             autoMode = AutoMode.AUTO_MODE_LEFT;
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
      while (!isStopRequested() && (gamepad1.a || gamepad1.b)) {
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
       
       /********************/
       /* Map All Motors   */
       /********************/
        FLMoto = hardwareMap.dcMotor.get("FLMoto");
        FRMoto = hardwareMap.dcMotor.get("BRMoto");
        BLMoto = hardwareMap.dcMotor.get("BLMoto");
        BRMoto = hardwareMap.dcMotor.get("FRMoto");
        
        liftMoto = hardwareMap.dcMotor.get("liftMoto");
        liftMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMoto.setTargetPosition(0);
        
        RS_Claw = hardwareMap.servo.get("rsClaw");
        LS_Claw = hardwareMap.servo.get("lsClaw");
        
        distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
        

        VoltSens = hardwareMap.voltageSensor.get("Control Hub");
       
       /********************/
       /* Map All Servos   */
       /********************/
       
       
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
       RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
       RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

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
      
       // Initialize Vuforia
       initVuforia ();
       initTfod();
       
       /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
       
       // Put initialization blocks here.
       
       // set direction of motors
        FLMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        FRMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        BLMoto.setDirection(DcMotorSimple.Direction.REVERSE);
        BRMoto.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Set the slider motor to the bottom before start 
        
        liftMoto.setDirection(DcMotorSimple.Direction.FORWARD); 
        
        liftMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // liftMoto.setTargetPosition(0);
        
        // Set the claw to the open position so the robot will fit into the 18" box
        RS_Claw.setDirection(Servo.Direction.FORWARD);
        LS_Claw.setDirection(Servo.Direction.FORWARD);
        openClaw();
        
        //Distance Sensor

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        
        //BlinkinLEDs
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
        blinkinLedDriver.setPattern(BasePattern);
        
        // telemetry.addData("Pattern: ", BasePattern.toString());
        // telemetry.update();
       
       
       // Create local variable to store selected automous mode
       // and amount of delay time
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
       boolean firstLoop = true;
       
       //waitForStart();
       while (!opModeIsActive() && !isStopRequested()) {
            findParkLoc();
       }
       
       while (opModeIsActive() && !isStopRequested() && firstLoop){
           tfod.shutdown(); 
           firstLoop = false;
       
           // set lights to parklocation detected
           switch (parkLocationDetected ) {
              case LOC_LOCATION1:
                  blinkinLedDriver.setPattern(Park1Pattern);

                  break;
 
              case LOC_LOCATION2:
                  blinkinLedDriver.setPattern(Park2Pattern);

                  break;
 
              case LOC_LOCATION3:
                  blinkinLedDriver.setPattern(Park3Pattern);

                  break;
 
              case LOC_NOT_SELECTED:
                 // This one should not happen if it does error and just stay where you are
                 blinkinLedDriver.setPattern(Park2Pattern);
                 break;
           }
       
           telemetry.clearAll();               // clear display messages

           telemetry.addData("Mode", "running");
           telemetry.update();

           resetEncoders();

           // close claw around preloaded cone
           closeClaw();
           sleep(300);

           // lift cone off of floor
           sliderPosition(posLowTerm,sleepTimeHigh);
       
           // Move forward away for the wall 1 inch at half power
           driveForward(0.5, quarterPower);
       
           // Call function to use TensorFlow to find parking location
           //findParkLoc();
       
           //driveBack(4.5, quarterPower);
       
           // Delay start if needed
           if (delayTimeMilliseconds > 0) {
              telemetry.addData("Status","Delaying");    // display delay status
              telemetry.update();
          
              sleep(delayTimeMilliseconds);                       // wait selected amount of time              
          
              telemetry.addData("Status","Running");  // dispay delay over and automous code is running
              telemetry.update();

           }

           // Determind which automous code to run
           switch (autoMode) {
              case AUTO_MODE_RIGHT:
                  RightAuto();
                  break;
 
              case AUTO_MODE_LEFT:
                  LeftAuto();
                  break;
 
             case AUTO_MODE_NOT_SELECTED:
                 // This one should not happen if it does do nothing
                 break;
           }         
       } 
    }
    
    /***************************************************************************/
    /* Function: Left Automous                                                 */
    /* Returns: none                                                           */
    /* Uses Global Variables: parkLocation                                     */
    /*                                                                         */
    /* This function is for when the robot starts on the left side of the      */
    /* field.                                                                  */
    /*      Red Zone - Tile F2 (near audience)                                 */
    /*      Blue Zone - Tile A5 (away from audience)                           */
    /* The robot will place the preloaded cone on the low pole and then park   */
    /***************************************************************************/
    private void LeftAuto() {
       
       // move to the right to get in front of low pole
       strafeRight(12.0, quarterPower);
       
       //drive toward pole
       driveForward(0.9, quarterPower);

       placeConeLow();
       
       // move to the right to center of tile F3/A4              
       strafeRight(15.0, halfPower);
       sliderPosition(posBottom,sleepTimeNone);
       
       //drive forward to line up with stacked cones
       driveForward(57.0, halfPower);
       
       //turning to face cones
       rotate(61, halfPower);
       
       //driving forward to get closer to cones
       driveForward(42.0, halfPower);
       
       //raising up slider to first cone before getting to cones
       sliderPosition(posFirstCone,sleepTimeMed);
       
       while (!(distSensor.getDistance(DistanceUnit.CM) > 0.5)) {
           resetEncoders();
           setDrivePower(quarterPower,quarterPower,quarterPower,quarterPower);
           
       }
       
       setDrivePower(noPower,noPower,noPower,noPower);
           
       //driveForward(0.5,0.2);
       //driveForward(0.5,0.1);
       //driveForward(0.5,0.2);
       //driveForward(2.0,0.1);
       
       //raising up slider to cones
       //sliderPosition(posFirstCone,sleepTimeFirst);
       
       //slowly driving in range of the cones
       //driveForward(4.0, 0.2);
       
       //grasping cone
       closeClaw();
       sleep(400);
       
       //lifting cone off stack
       sliderPosition(posFirstCone-1000,sleepTimeFirst);
       sleep(100);
       
       //backing away from stack of cones
       driveBack(1.0,quarterPower);

       //**********************************
       // Place Low Pole
       //*********************************
       
       // drive back to low pole
       driveBack(3.0,halfPower);
       
       //slider raises to low pole height
       sliderPosition(posLowTerm,sleepTimeLow);
       
       //turning to face low pole
       rotate(56, halfPower);
       
       //drops cone onto pole
       openClaw();
       
       //lower slider slightly onto pole
       sliderPosition(posLowTerm + 600,sleepTimeLow);
       
       // return to get second cone
       rotate(-56, halfPower);
       
       //lower slider to cones
       sliderPosition(posSecondCone,sleepTimeNone);
     
       //driving forward to get closer to cones
       driveForward(10.5, 0.35);
       //driveForward(3.0,0.3);
       //driveForward(2.0,0.2);
       
       while (!(distSensor.getDistance(DistanceUnit.CM) > 0.7)) {
           resetEncoders();
           setDrivePower(quarterPower,quarterPower,quarterPower,quarterPower);
           
       }
       
       setDrivePower(noPower,noPower,noPower,noPower);
       
       //grasping cone
       closeClaw();
       sleep(500);
       
       //lifting cone off stack
       sliderPosition(posFirstCone-1000,sleepTimeFirst);
       sleep(100);
       
       //backing away from stack of cones
       driveBack(1.0,quarterPower);
       
       /**********************************
       // Place Medium Pole
       /*********************************/
       
       // drive back to medium pole
       driveBack(26.0,halfPower);
       
       //slider raises to medium pole
       sliderPosition(posMediumTerm,sleepTimeMed);
       
       //turning to face medium pole
       rotate(57, halfPower);
       
       //backing away from pole
       //driveBack(0.5,quarterPower);
      
       //driveForward(0.5, quarterPower);
       
       //lower slider slightly onto pole
       sliderPosition(posMediumTerm + 600,sleepTimeMed);
       
       //drops cone onto pole
       openClaw();
   
       // park in correct location
       switch (parkLocationDetected ) {
          case LOC_LOCATION1:
              // move to the left to center of tile F1/A6              
              strafeRight(50.0, halfPower);
              
              // drive forward to park in location zone
              driveForward(10.0, halfPower);

              break;
 
          case LOC_LOCATION2:
              // move to the left to center of tile F2/A5              
              strafeRight(15.0, halfPower);
              
              closeClaw();
              
               // drive forward to park in location zone
              driveForward(2.0, halfPower);

              break;
 
          case LOC_LOCATION3:
              // move to the right to center of tile F3/A4              
              strafeLeft(13.0, halfPower);
       
              // drive forward to park in location zone
              driveForward(10.0, halfPower);

              break;
 
          case LOC_NOT_SELECTED:
             // This one should not happen if it does error and just stay where you are
             break;
       }

       // lower slider
       sliderPosition(posBottom,sleepTimeHigh + 500);
       

    } // End of LeftAuto
    
    /***************************************************************************/
    /* Function: Right Auto                                                    */
    /* Returns: none                                                           */
    /* Uses Global Variables: parkLocation                                     */
    /*                                                                         */
    /* This function is for when the robot starts on the right side of the     */
    /* field.                                                                  */
    /*      Red Zone - Tile F5 (away from audience)                            */
    /*      Blue Zone - Tile A2 (near      audience)                           */
    /* The robot will place the preloaded cone on the low pole and then park   */
    /***************************************************************************/
    private void RightAuto() {
        
       //drive back from pole
       //driveBack(4.0, quarterPower);
       
       // move to the left to get in front of low pole
       //strafeLeft(19.0, quarterPower);
       strafeLeft(20.0, quarterPower);
       //sleep(200);

       //(0.1, quarterPower);
       //sleep(100);

      // driveForward(1.0, quarterPower);
       
       placeConeLow();
       
       driveBack(2.0, quarterPower);
       sleep(100);

       // move to the left to center of tile F3/A4              
       //driveBack(1.0, quarterPower);
       strafeLeft(12.0, halfPower);
       sliderPosition(posBottom,sleepTimeNone);

       //drive forward to line up with stacked cones
       driveForward(58.5, halfPower);
       
       //rotate robot to face stacked cones
       rotate(-62, halfPower);
       
       //drive forward to approach cone
       driveForward(40.0, halfPower);
       
       //raise slider to lineup with stacked cone
       sliderPosition(posFirstCone,sleepTimeFirst);
       
       //drive forward to pick up cone
       driveForward(4.0,0.3);
       //driveForward(2.0,0.2);
       //driveForward(0.3,0.1);

      while (!(distSensor.getDistance(DistanceUnit.CM) > 0.5)) {
           resetEncoders();
           setDrivePower(quarterPower,quarterPower,quarterPower,quarterPower);
       }

       //grab cone off stack
       closeClaw();
       sleep(300);
       
      //lifting cone off stack
       sliderPosition(posFirstCone-1000,sleepTimeFirst);
       //sleep(100);
      
       //drive back from stacked cones
       driveBack(1.0,quarterPower);
       sleep(100);
       
       
       // drive back to low pole
       driveBack(4.0,halfPower);
       
       //slider raises to low pole height
       sliderPosition(posLowTerm,sleepTimeLow);
       
       //turning to face low pole
       rotate(-56, halfPower);
       
       //lower slider slightly onto pole
       sliderPosition(posLowTerm + 600,sleepTimeLow);
       
       //drops cone onto pole
       openClaw();
       sleep(100);
       
       // return to get second cone
       rotate(56, halfPower);
       
       //lower slider to cones
       sliderPosition(posSecondCone,sleepTimeNone);
     
       //driving forward to get closer to cones
       driveForward(7.0, 0.4);
       
       while (!(distSensor.getDistance(DistanceUnit.CM) > 0.7)) {
           resetEncoders();
           setDrivePower(quarterPower,quarterPower,quarterPower,quarterPower);
           
       }

       //driveForward(1.0,0.4);
       //driveForward(1.0,0.3);
       //driveForward(1.0,0.2);
       
       //grasping cone
       closeClaw();
       sleep(500);
       
      //lifting cone off stack
       sliderPosition(posSecondCone-1000,sleepTimeFirst);
       sleep(100);
       
       //backing away from stack of cones
       driveBack(1.0,quarterPower); 
       
    
        
       // place cone med pole
       driveBack(28.0,halfPower);
       
       //raise slider to height of medium pole
       sliderPosition(posMediumTerm,sleepTimeMed);
       
       //rotate robot to face pole
       rotate(-57, halfPower);
       
       //drive forward towards pole
       //driveForward(1.0, quarterPower);
       
       //lower slider onto pole
       sliderPosition(posMediumTerm + 600,sleepTimeMed);
       
       //driveBack(1.0, quarterPower);
       
       //drop cone
       openClaw();
   
       // park in correct location
       switch (parkLocationDetected ) {
          case LOC_LOCATION1:
              // move to the left to center of tile F1/A6              
              strafeRight(14.5, halfPower);
              
              // drive forward to park in locatio n zone
              driveForward(10.0, halfPower);

              break;
 
          case LOC_LOCATION2:
              // move to the left to center of tile F2/A5              
              strafeLeft(15.0, halfPower); 
              
              closeClaw();
              
              // drive forward to park in location zone
              driveForward(2.0, halfPower);

              break;
 
          case LOC_LOCATION3:
              // move to the right to center of tile F3/A4              
              strafeLeft(45.0, halfPower);
       
              // drive forward to park in location zone
              driveForward(10.0, halfPower);

              break;
 
          case LOC_NOT_SELECTED:
             // This one should not happen if it does error and just stay where you are
             break;
       }

       // lower slider
       sliderPosition(posBottom,sleepTimeHigh + 500);
       //sliderPosition(posBottom);
       
    }  // End of RightAuto
    
    /************************************************************/
    /* Function: sliderPosition                                */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to pick up the cone on the       */
    /* first stack                                              */
    /************************************************************/
    private void sliderPosition(int position,int sleepTime)
    {
      liftMoto.setTargetPosition(position);
      liftMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      liftMoto.setPower(1.0);
      sleep(sleepTime);
    }
    

    /************************************************************/
    /* Function: placeConeLow                                   */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called place the cone on one of the     */
    /* low poles from the start position.                       */
    /************************************************************/
    private void placeConeLow()
    {
       // raise slider to high of low pole
       sliderPosition(posLowTerm,sleepTimeHigh);
       
       //drive forward towards pole
       driveForward(1.0, quarterPower);
       sleep(200);
       
       sliderPosition(posLowTerm+500,sleepTimeLow);
       
       // open claw to drop cone
       openClaw();
       sleep(100);
       
       sliderPosition(posLowTerm-500,sleepTimeLow);
       
    }
 

    /************************************************************/
    /* Function: closeClaw                                      */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called set the servos to close the claw */
    /************************************************************/
    private void closeClaw() {
        RS_Claw.setPosition(0.85);
        LS_Claw.setPosition(0.55);
    }   
    
    /************************************************************/
    /* Function: openClaw                                       */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called set the servos to open the claw  */
    /************************************************************/
    private void openClaw()
    {
        RS_Claw.setPosition(0.6);
        LS_Claw.setPosition(0.8);
    }
    
    /************************************************************/
    /* Function: resetEncoders                                  */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to stop and reset the encoders   */
    /* on all 4 motors.                                         */
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
    /* This function is called to set all 4 motors to run using */
    /* encoders.                                                */
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
    /* This function is called to set the power of all 4 motors */
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
    private void driveForward(Double inches,Double power)
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
    private void driveBack(Double inches,Double power)
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
        // fr, bl, br, fl
        driveForwardInch(inches, -power-0.1, -power, power, power+0.2);
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
    
    /************************************************************/
    /*                                                          */
    /* Function: driveRightInch                                 */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move straigh   */
    /* in a forward or reverse direction.                       */
    /*                                                          */
    /************************************************************/
    private void driveRightInch(Double inches,Double FLpower,Double FRpower, Double BLpower, Double BRpower) {
       
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
    
        
    /***************************** initVuforia ***************************
     * Initialize the Vuforia localization engine.
     ********************************************************************/
    private void initVuforia() 
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

       // parameters.vuforiaLicenseKey = VUFORIA_KEY;
       // parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    
    /***************************** initTfod ******************************
     * Initialize the TensorFlow Object Detection engine.
     ********************************************************************/
    private void initTfod() 
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       // tfodParameters.minResultConfidence = 0.75f;
       // tfodParameters.isModelTensorFlow2 = true;
       // tfodParameters.inputSize = 300;
       // tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        
        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        // tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        //tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        
    }


    /***************************** findParkLoc ******************************
     * Uses Tensor Flow to determine the parking location
     ********************************************************************/
    private void findParkLoc() {
       
        // blinkinLedDriver.setPattern(BasePattern);
      
        // park location = LOC_NOT_SELECTED, LOC_LOCATION1, LOC_LOCATION2, LOC_LOCATION3  
        // set park location to 1 in case the cone is not detected since the bolts are
        // missed the most.
        parkLocationDetected = parkLocation.LOC_LOCATION2;
        
        boolean isConeDetected = false;
      
        sleep (100);
      
          if (tfod != null) 
        {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    
            if (updatedRecognitions != null) 
            {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                    
                for (Recognition recognition : updatedRecognitions) 
                {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    i++;
                                 
                    if (recognition.getLabel().equals("yellowCookie")) {
                        telemetry.addData("Parking Location", " 1 Yellow Cookie");
 
                        isConeDetected = true;
                        parkLocationDetected = parkLocation.LOC_LOCATION1;
                        telemetry.update();
  
                        return;

                    }
                    
                    if (recognition.getLabel().equals("blueSquare")) {
                        telemetry.addData("Parking Location", " 2 Blue Square");
 
                        isConeDetected = true;
                        parkLocationDetected = parkLocation.LOC_LOCATION2;
                        
                        telemetry.update();
                        
                        return;

                    }

                    if (recognition.getLabel().equals("pinkTriangle")) {
                        telemetry.addData("Parking Location", " 3 Pink Triangle");
 
                        isConeDetected = true;
                        parkLocationDetected = parkLocation.LOC_LOCATION3;
                        
                        telemetry.update();
                        
                        return;

                    }                                                       
                }

            } else 
            {
                isConeDetected = false;
                telemetry.addData("Object Detected", "NONE"); 
                
                // set default 
                blinkinLedDriver.setPattern(Park2Pattern);
            }
                
            telemetry.update();
            //tfod.shutdown();  
                      
        }
          
    }
  
}