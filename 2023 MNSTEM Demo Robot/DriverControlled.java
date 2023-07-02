package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "DriverControlled", group = "DriverControlled")

public class DriverControlled extends OpMode {

    RobotHardware robot = new RobotHardware();

   BNO055IMU imu;
        Orientation angles;
        Acceleration gravity;

  public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
      //  parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
       }

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("STATUS", "Initialized");
    
       
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
  
        initGyro();  
  
    }

    @Override
    public void loop() {

        boolean G1Right = gamepad1.b;
        boolean G1Left = gamepad1.x;
        boolean G1DpadLeft = gamepad1.dpad_left;
        boolean G1DpadRight = gamepad1.dpad_right;
        boolean G1DpadUp = gamepad1.dpad_up;
        boolean G1DpadDown = gamepad1.dpad_down;
        double G1leftStickY = -gamepad1.left_stick_y;
        double G1leftStickX = gamepad1.left_stick_x;
        double G1rightStickY = -gamepad1.right_stick_y;
        double G1rightStickX = gamepad1.right_stick_x;
        boolean G1leftBumper = gamepad1.left_bumper;
        boolean G1rightBumper = gamepad1.right_bumper;
        double G1rightTrigger = gamepad1.right_trigger;
        double G1leftTrigger = -gamepad1.left_trigger;
        double AdjustSpeed = 0.3;
      
        // Adjust speed here
        // Adjust speed here
        double RobotSpeed = 0.8;
        // Adjust speed here
        // Adjust speed here

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          double  theta = angles.firstAngle;
            
            

        if (G1Left) {

            robot.LeftFront.setPower(-AdjustSpeed);
            robot.LeftBack.setPower(AdjustSpeed);
            robot.RightFront.setPower(-AdjustSpeed);
            robot.RightBack.setPower(AdjustSpeed);

        } else if (G1Right) {

            robot.LeftFront.setPower(AdjustSpeed);
            robot.LeftBack.setPower(-AdjustSpeed);
            robot.RightFront.setPower(AdjustSpeed);
            robot.RightBack.setPower(-AdjustSpeed);

        } else if (G1DpadLeft) {

            robot.LeftFront.setPower(-0.4);
            robot.LeftBack.setPower(-0.4);
            robot.RightFront.setPower(-0.4);
            robot.RightBack.setPower(-0.4);

        } else if (G1DpadRight) {

            robot.LeftFront.setPower(0.4);
            robot.LeftBack.setPower(0.4);
            robot.RightFront.setPower(0.4);
            robot.RightBack.setPower(0.4);

        } else if (G1DpadUp) {

            robot.LeftFront.setPower(AdjustSpeed);
            robot.LeftBack.setPower(-AdjustSpeed);
            robot.RightFront.setPower(-AdjustSpeed);
            robot.RightBack.setPower(AdjustSpeed);

        } else if (G1DpadDown) {

            robot.LeftFront.setPower(-AdjustSpeed);
            robot.LeftBack.setPower(AdjustSpeed);
            robot.RightFront.setPower(AdjustSpeed);
            robot.RightBack.setPower(-AdjustSpeed);

        } else if (G1leftBumper || G1rightBumper) {
            
            if (G1leftStickY > 0.975 || G1leftStickY < -0.975){

                robot.LeftFront.setPower(G1leftStickY/1.25);
                robot.LeftBack.setPower(-(G1leftStickY/1.25));

            } else {

                robot.LeftFront.setPower(G1leftStickY/1.75);
                robot.LeftBack.setPower(-(G1leftStickY/1.75));

            }

            if (G1rightStickY > 0.975 || G1rightStickY < -0.975){

                robot.RightFront.setPower(-(G1rightStickY/1.25));
                robot.RightBack.setPower(G1rightStickY/1.25);

            } else {

                robot.RightFront.setPower(-(G1rightStickY)/1.75);
                robot.RightBack.setPower(G1rightStickY/1.75);

            }


        } else if (G1rightTrigger > 0.1) {

            if (G1rightTrigger > 0.975){

                robot.LeftFront.setPower(G1rightTrigger/1.25);
                robot.LeftBack.setPower(G1rightTrigger/1.25);
                robot.RightFront.setPower(G1rightTrigger/1.25);
                robot.RightBack.setPower(G1rightTrigger/1.25);

            } else {

                robot.LeftFront.setPower(G1rightTrigger/1.75);
                robot.LeftBack.setPower(G1rightTrigger/1.75);
                robot.RightFront.setPower(G1rightTrigger/1.75);
                robot.RightBack.setPower(G1rightTrigger/1.75);

            }

        } else if (G1leftTrigger < -0.1) {

            if (G1leftTrigger < -0.975){

                robot.LeftFront.setPower(G1leftTrigger/1.25);
                robot.LeftBack.setPower(G1leftTrigger/1.25);
                robot.RightFront.setPower(G1leftTrigger/1.25);
                robot.RightBack.setPower(G1leftTrigger/1.25);

            } else {

                robot.LeftFront.setPower(G1leftTrigger/1.75);
                robot.LeftBack.setPower(G1leftTrigger/1.75);
                robot.RightFront.setPower(G1leftTrigger/1.75);
                robot.RightBack.setPower(G1leftTrigger/1.75);

            }

        } else {
            
            double x1 = G1leftStickY;
            double y1 = G1leftStickX * 1.0;
            double rx = G1rightStickX;
            
          double  x = x1 * (Math.cos(theta)) - y1 * (Math.sin(theta));
          double y = x1 * (Math.sin(theta)) + y1 * (Math.cos(theta));
            
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x - rx) / denominator;
            double frontRightPower = (y - x + rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            
            robot.LeftFront.setPower(frontLeftPower * RobotSpeed);
            robot.LeftBack.setPower(backLeftPower * RobotSpeed);
            robot.RightFront.setPower(frontRightPower * RobotSpeed);
            robot.RightBack.setPower(backRightPower * RobotSpeed);

        }
    }

    @Override
    public void stop() {
    }
    
  
}
