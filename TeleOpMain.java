//Import required librarys
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.regex.Pattern;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TeleOpMain", group = "Drive")
public class TeleOpMain extends LinearOpMode {
  
  //Drive Wheels
  private DcMotor FLMoto;
  private DcMotor FRMoto;
  private DcMotor BLMoto;
  private DcMotor BRMoto;
  
  
  //Hanging Linear Actuator
  private DcMotor hangMoto;
  private DcMotor armMoto;
  
  
  //Intake
  private DcMotor intakeArm;
  private DcMotor intakeArmRaise;
  private Servo intakeServoLeft;
  private Servo intakeServoRight;
  private Servo intakeRotateServo;

  //BlinkinLEDs
  private RevBlinkinLedDriver blinkinLedDriver;
  private RevBlinkinLedDriver.BlinkinPattern BasePattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
  private RevBlinkinLedDriver.BlinkinPattern StopPattern = RevBlinkinLedDriver.BlinkinPattern.RED;


  //Power Constant
  private static double powerConstant = .51;

  protected enum DisplayKind {
    MANUAL,
    AUTO
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    double Voltage = 0;
    double wUP = .3;
    double Iin = 1;
    double Fly = 1;

    
    //****************************************//
    // Map all robot hardware                 //
    //****************************************//   

    //****************************************//
    //BR means "back right"                   //
    //BL means "back left"                    //
    //FR means "front right"                  //
    //FL means "front left"                   //
    //****************************************//

    FLMoto = hardwareMap.dcMotor.get("FLMoto");
    FRMoto = hardwareMap.dcMotor.get("BRMoto");
    BLMoto = hardwareMap.dcMotor.get("BLMoto");
    BRMoto = hardwareMap.dcMotor.get("FRMoto");
    hangMoto = hardwareMap.dcMotor.get("hangMoto");
    armMoto = hardwareMap.dcMotor.get("armMoto");
    intakeArm = hardwareMap.dcMotor.get("intakeArm");
    intakeServoLeft = hardwareMap.servo.get("intakeServoLeft");
    intakeServoRight = hardwareMap.servo.get("intakeServoRight");
    intakeRotateServo = hardwareMap.servo.get("intakeRotateServo");
    intakeArmRaise = hardwareMap.dcMotor.get("intakeArmRaise");
    
    
    //BlinkinLEDs
    blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
    blinkinLedDriver.setPattern(BasePattern);
    
    int armMotoPosition = 0;

    telemetry.update();

    float hsvValues[] = {
      0F,
      0F,
      0F
    };
    final float values[] = hsvValues;

    boolean aPressed = false;
    boolean bPressed = false;
    boolean yPressed = false;
    boolean xPressed = false;
    
    //*****************************************//
    // Put initialization blocks here.         //
    //*****************************************//

    //***************************************************//
    // Set direction of all motors                       //
    //***************************************************//
    
    FLMoto.setDirection(DcMotorSimple.Direction.FORWARD);
    FRMoto.setDirection(DcMotorSimple.Direction.REVERSE);
    BLMoto.setDirection(DcMotorSimple.Direction.FORWARD);
    BRMoto.setDirection(DcMotorSimple.Direction.REVERSE);
    
    //armRMoto Rotates the lifting arm
    armMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armMoto.setDirection(DcMotorSimple.Direction.FORWARD);
    //hangMoto moves the lift arm up and down
    //hangMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armMoto.setDirection(DcMotorSimple.Direction.FORWARD);
    
    //hangMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //hangMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //armMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    //hangMoto.setTargetPosition(0);
    resetEncoders();
    // Wait for the start of TeleOp
    waitForStart();

    // Put run blocks here.

    //******************************************//
    // Run code while op mode is active         //
    //******************************************//
    while (opModeIsActive()) {
      //telemetry stuff
      telemetry.addData("Status", "opModeIsActive");
      telemetry.addData("Lift Rotation:", armMoto.getCurrentPosition());
      telemetry.update();
      
      if(gamepad2.a)
      {
        aPressed = true;
        bPressed = false;
        yPressed = false;
        xPressed = false;
      }
      
      if(gamepad2.b)
      {
        aPressed = false;
        bPressed = true;
        yPressed = false;
        xPressed = false;
      }
      
      if(gamepad2.y)
      {
        aPressed = false;
        bPressed = false;
        yPressed = true;
        xPressed = false;
      }
      
      if(gamepad2.x){
        aPressed = false;
        bPressed = false;
        yPressed = false;
        xPressed = true;
      }
      //Rotate hang arm to attachPole position
      if(aPressed == true)
      {
        moveToPole();
      }
      
      //Rotate hang arm to approchingPole Positon
      if(bPressed == true)
      {
        moveToApproachingPositon();
      }
      
      //Rotate hang arm to bottom positon
      if(yPressed == true)
      {
        moveToBottomPosition();
      }
      
      if(xPressed == true){
        allTheWayDown();
      }
      telemetry.update();
      
      //Code for rotating hang actuator
      //armMoto.setPower(gamepad2.left_stick_y/3);

        
      // code for raising hang actuator
        if(gamepad2.right_bumper == true){
          hangMoto.setPower(1);
        }
        else if(gamepad2.left_bumper == true){
          hangMoto.setPower(-1);
        }
        else{
          hangMoto.setPower(0);
        }
        
        //intake arm rotation
        intakeArm.setPower(gamepad2.right_stick_y/2);
        
        //Intake Servo mapping
        /*
        if(gamepad2.dpad_up == true)
        {
          removePixel();
        }
        
        if(gamepad2.dpad_down == true)
        {
          takeInPixel();
        }
        
        if(gamepad2.dpad_up == true){
          
        }
        */
        //Include Regular Drive Mechanics
        FRMoto.setPower(gamepad1.right_stick_y); //FL
        FLMoto.setPower(gamepad1.left_stick_y); //BR
        BRMoto.setPower(gamepad1.right_stick_y); //BL
        BLMoto.setPower(gamepad1.left_stick_y); //FR
        
        //Strafe Right
        FRMoto.setPower(gamepad1.right_trigger);
        FLMoto.setPower(gamepad1.right_trigger);
        BRMoto.setPower(-gamepad1.right_trigger);
        BLMoto.setPower(-gamepad1.right_trigger);
        
        // Strafe Left
        FRMoto.setPower(-gamepad1.left_trigger);
        FLMoto.setPower(-gamepad1.left_trigger);
        BRMoto.setPower(gamepad1.left_trigger);
        BLMoto.setPower(gamepad1.left_trigger);
      
        telemetry.update();
    }
      //if(gamepad2.a)
      // {
      //   moveToPole();
      // }
  }

      
      private void resetEncoders() 
      {
       armMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }
      
      public void moveToPole()
      {
        armMoto.setTargetPosition(-158);
        armMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMoto.setPower(0.7);
        telemetry.addData("Lift Rotation:", "a button pressed");
      }
      
      public void moveToBottomPosition()
      {
        armMoto.setTargetPosition(0);
        armMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMoto.setPower(1);
        telemetry.addData("Lift Rotation:", "y button pressed");
      }
      
      public void moveToApproachingPositon()
      {
        armMoto.setTargetPosition(-208);
        armMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMoto.setPower(1);
        telemetry.addData("Lift Rotation:", "b button pressed");
      }
      
      public void allTheWayDown(){
        armMoto.setTargetPosition(-195);
        armMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMoto.setPower(1);
        telemetry.addData("Lift Rotation:", "b button pressed");
      }
      /*
      public void takeInPixel()
      {
        intakeServoLeft.setPosition(1);
        intakeServoRight.setPosition(0);
      }
      
      public void removePixel()
      {
        intakeServoLeft.setPosition(0);
        intakeServoRight.setPosition(1);
      }
      */
    
}