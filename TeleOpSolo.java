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


@TeleOp(name = "TeleOpSolo", group = "Drive")
public class TeleOpSolo extends LinearOpMode {


  
  //Drive Wheels
  private DcMotor FLMoto;
  private DcMotor FRMoto;
  private DcMotor BLMoto;
  private DcMotor BRMoto;
  
  //Hanging Linear Actuator
  private DcMotor hangMoto;
  private DcMotor hangRMoto;
  
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
    hangRMoto = hardwareMap.dcMotor.get("hangRMoto");
    //distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
    
    //BlinkinLEDs
    blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
    blinkinLedDriver.setPattern(BasePattern);
    
    int hangMotoPosition = 0;

    telemetry.update();

    float hsvValues[] = {
      0F,
      0F,
      0F
    };
    final float values[] = hsvValues;

    //*****************************************//
    // Put initialization blocks here.         //
    //*****************************************//

    //***************************************************//
    // Set direction of all motors                       //
    //***************************************************//
    
    FLMoto.setDirection(DcMotorSimple.Direction.REVERSE);
    FRMoto.setDirection(DcMotorSimple.Direction.FORWARD);
    BLMoto.setDirection(DcMotorSimple.Direction.REVERSE);
    BRMoto.setDirection(DcMotorSimple.Direction.FORWARD);
    
    //setting the brake behavior
    FLMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FRMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BLMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BRMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    //hangRMoto Rotates the lifting arm
    //hangMoto moves the lift arm up and down
    //hangMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    hangRMoto.setDirection(DcMotorSimple.Direction.FORWARD);
    
    hangMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    hangMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    hangMoto.setTargetPosition(0);
    
    // Wait for the start of TeleOp
    waitForStart();

    // Put run blocks here.

    //******************************************//
    // Run code while op mode is active         //
    //******************************************//
    while (opModeIsActive()) {
      telemetry.addData("Status", "opModeIsActive");

      telemetry.update();

     // telemetry.addData("hang Position:", hangMotoPosition);
      

      //Loop Blocks
      //hangMoto.setTargetPosition(hangMotoPosition);
    
      
      //Code for rotating hang actuator
        hangRMoto.setPower(gamepad2.left_stick_y/2);
        //hangRMoto.setPower(gamepad2.left_stick_y);
        
      // code for raising hang actuator
        if(gamepad1.dpad_up == true){
          hangMoto.setPower(1);
        }
        else if(gamepad1.dpad_down == true){
          hangMoto.setPower(-1);
        }
        else{
          hangMoto.setPower(0);
        }
        
      
        //Include Regular Drive Mechanics
        FRMoto.setPower(gamepad1.left_stick_y); //FL
        FLMoto.setPower(gamepad1.right_stick_y); //BR
        BRMoto.setPower(gamepad1.left_stick_y); //BL
        BLMoto.setPower(gamepad1.right_stick_y); //FR
        
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
  }
}