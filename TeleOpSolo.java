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

  //public boolean aButt = false;
  
  //Drive Wheels
  private DcMotor FLMoto;
  private DcMotor FRMoto;
  private DcMotor BLMoto;
  private DcMotor BRMoto;
  
  //Hanging Linear Actuator
  private DcMotor hangMoto;
  private DcMotor armMoto;
  
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
    //distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
    
    
    //BlinkinLEDs
    blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLed");
    blinkinLedDriver.setPattern(BasePattern);
    
    int armMotoPosition = 0;
    int speedMod = 1;

    //int pastPole = 297;
    //int atPole = 220;
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
    
    //armRMoto Rotates the lifting arm
    armMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armMoto.setDirection(DcMotorSimple.Direction.FORWARD);
    //hangMoto moves the lift arm up and down
    hangMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armMoto.setDirection(DcMotorSimple.Direction.FORWARD);
    
    hangMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    hangMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //armMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    resetEncoders();
    // Wait for the start of TeleOp
    waitForStart();

    // Put run blocks here.

    //******************************************//
    // Run code while op mode is active         //
    //******************************************//
    while (opModeIsActive()) {
      telemetry.addData("Status", "opModeIsActive");
      telemetry.addData("Lift Rotation:", armMoto.getCurrentPosition());
      telemetry.update();

     // telemetry.addData("hang Position:", hangMotoPosition);
      
      //Loop Blocks
      while(gamepad1.x){
        speedMod=1.5;
      }
      /*else{
        speedMod=1;
      }*/
      if(gamepad1.a)
      {
      armMoto.setTargetPosition(220);
      armMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      armMoto.setPower(1);
      telemetry.addData("Lift Rotation:", "a button pressed");
      sleep(100);
      }
      
      if(gamepad1.b)
      {
      armMoto.setTargetPosition(297);
      armMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      armMoto.setPower(1);
      telemetry.addData("Lift Rotation:", "a button pressed");
      sleep(100);
      }
      
      
      if(gamepad1.y)
      {
      armMoto.setTargetPosition(0);
      armMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      armMoto.setPower(1);
      telemetry.addData("Lift Rotation:", "a button pressed");
      sleep(100);
      }
      
      telemetry.update();
      //Code for rotating hang actuator
      //armMoto.setPower(gamepad1.left_stick_y/3);

        
      // code for raising hang actuator
        if(gamepad1.right_bumper == true){
          hangMoto.setPower(1);
        }
        else if(gamepad1.left_bumper == true){
          hangMoto.setPower(-1);
        }
        else{
          hangMoto.setPower(0);
        }
        
      
        //Include Regular Drive Mechanics
        FRMoto.setPower(gamepad1.left_stick_y*speedMod); //FL
        FLMoto.setPower(gamepad1.right_stick_y*speedMod); //BR
        BRMoto.setPower(gamepad1.left_stick_y*speedMod); //BL
        BLMoto.setPower(gamepad1.right_stick_y*speedMod); //FR
        
        //Strafe Right
        FRMoto.setPower(gamepad1.right_trigger*speedMod);
        FLMoto.setPower(gamepad1.right_trigger*speedMod);
        BRMoto.setPower(-gamepad1.right_trigger*speedMod);
        BLMoto.setPower(-gamepad1.right_trigger*speedMod);
        
        // Strafe Left
        FRMoto.setPower(-gamepad1.left_trigger*speedMod);
        FLMoto.setPower(-gamepad1.left_trigger*speedMod);
        BRMoto.setPower(gamepad1.left_trigger*speedMod);
        BLMoto.setPower(gamepad1.left_trigger*speedMod);
      
        telemetry.update();
    }
  }

      
      private void resetEncoders() 
      {
       armMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }
    
  }
    }
      
      
    
}