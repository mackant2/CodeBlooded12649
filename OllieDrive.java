package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class OllieDrive extends LinearOpMode {
   @Override
   public void runOpMode() throws InterruptedException {
      // Declare our motors
      // Make sure your ID's match your configuration
      DcMotor motorFrontLeft = hardwareMap.dcMotor.get("left_front");
      DcMotor motorBackLeft = hardwareMap.dcMotor.get("left_back");
      DcMotor motorFrontRight = hardwareMap.dcMotor.get("right_front");
      DcMotor motorBackRight = hardwareMap.dcMotor.get("right_back");
      DcMotor Duckspinner = hardwareMap.dcMotor.get("Duckspinner");
      DcMotor Light = hardwareMap.dcMotor.get("Light");
      DcMotor Arm = hardwareMap.dcMotor.get("Arm");
      // Reverse the right side motors
      // Reverse left motors if you are using NeveRests
      motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
      motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

      waitForStart();
      Light.setPower(1);
      while (opModeIsActive()) {
        double left;
        double right;
        
        double leftDrivePower;
        double rightDrivePower;
        double leftfrontPower;
        double rightfrontPower;
        // Run wheels in tank mode (note: The joystick goes pos when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;

        leftDrivePower = left;
        
       rightDrivePower = right;
         
         right = gamepad1.right_stick_y;
          left = gamepad1.left_stick_y;
        
       
      leftfrontPower = left;
      rightfrontPower  = right;
       
         left = gamepad1.left_stick_x;
         right = gamepad1.right_stick_x;

        
         leftDrivePower = leftDrivePower + left;
        rightDrivePower = rightDrivePower + right;
        
        
       left = gamepad1.left_stick_x;
       right = gamepad1.right_stick_x;
      
      leftfrontPower = leftfrontPower + right;
        rightfrontPower = rightfrontPower + left;
     
     
      motorBackLeft.setPower(leftDrivePower);
      motorBackRight.setPower(rightDrivePower);
      motorFrontLeft.setPower(leftfrontPower);
      motorFrontRight.setPower(rightfrontPower);
      

         
        if (gamepad1.right_trigger>.2)
            Duckspinner.setPower(-gamepad1.right_trigger);
         else if (gamepad1.left_trigger>.2)
          Duckspinner.setPower(1*gamepad1.left_trigger);
          else  
          Duckspinner.setPower(0);
          
          if (gamepad2.left_bumper) {
                Arm.setPower(.1);
        } else if (gamepad2.right_bumper) {
            Arm.setPower(-.1);
        } else {
            Arm.setPower(0);
        }
      }
   
       
   }


}


