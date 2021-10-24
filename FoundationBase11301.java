package org.firstinspires.ftc.teamcode1920;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.io.FileWriter;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.io.FileWriter;  

import java.util.Locale;

@Autonomous(name="Foundation Base 11301", group="Base")

public class FoundationBase11301 extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    /* Declare OpMode members. */
    RevRobotHardwareNew         robot   = new RevRobotHardwareNew();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1300 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    int ArmPosition;
    int LoopNum = 0;
    double time = 1;
    double speed;
    double heading;
    double TopEncoderPos;
    boolean                 Red = false;
    boolean                 Blue = true;
    boolean                 Alliance;
    double                  pi = Math.acos(-1);
    double                  x;
    double                  y;
    double                  xLocation = 40; 
    double                  yLocation = 62;
    double                  LocationEncoderT;
    double                  LocationEncoderB;
    double                  LocationEncoderL;
    double                  LocationEncoderR;
    double                  LocationEncoderT1;
    double                  LocationEncoderB1;
    double                  LocationEncoderL1;
    double                  LocationEncoderR1;
    double                  rotationCorrection;
    double                  EncoderPositionT;
    double                  EncoderPositionB;
    double                  EncoderPositionL;
    double                  EncoderPositionR;
    double                  TPower;
    double                  BPower;
    double                  LPower;
    double                  RPower;
    double                  leftpower;
    double                  rightpower;
    double                  toppower;
    double                  bottompower;
    boolean                 Block = false;
    boolean                 WriteLog = true;
    String                  FName = "/storage/emulated/0/download/testout.txt";
    double                  xTarget;
    double                  yTarget;
    int                     i;
    boolean                 Stone;
    double                  PullPos1;
    double                  PullPos2;
    private ElapsedTime     opmodeRunTime = new ElapsedTime();

    Servo   servo;
    @Override
    public void runOpMode() {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        BNO055IMU.Parameters parametersGyro = new BNO055IMU.Parameters();
        parametersGyro.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parametersGyro.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersGyro.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersGyro.loggingEnabled      = true;
        parametersGyro.loggingTag          = "IMU";
        parametersGyro.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersGyro);
        
        robot.init(hardwareMap);
        
        servo = hardwareMap.get(Servo.class, "FoundPull");
        servo = hardwareMap.get(Servo.class, "FoundPull2");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.leftDrive.getCurrentPosition(),
                          robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        opmodeRunTime.reset(); 
        runtime.reset();
        // Lift Pulls Up
        robot.FoundPull.setPosition(1);
        robot.FoundPull2.setPosition(0);
        sleep(500);    
        telemetry.addData("Driving forward", " to Foundation");
        telemetry.update();
        robot.Claw.setPosition(0.2);
        robot.Light.setPower(0.5);
        robot.Capstone.setPosition(0.7);
        sleep(100);
        
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.TopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        LocationEncoderR = robot.rightDrive.getCurrentPosition();
        LocationEncoderL = robot.leftDrive.getCurrentPosition();
        LocationEncoderT = robot.TopDrive.getCurrentPosition();
        LocationEncoderB = robot.BottomDrive.getCurrentPosition();
        EncoderPositionR = robot.rightDrive.getCurrentPosition();
        while (!isStopRequested() && opModeIsActive()){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading=angles.firstAngle;
            if (Math.abs(yLocation) > 28.5 && Math.abs(yLocation) < 29.5 && 
                        Math.abs(xLocation) > 49.5 && Math.abs(xLocation) < 50.5 && heading < 3 && heading > -3){
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.TopDrive.setPower(0);
                robot.BottomDrive.setPower(0);
                break;          
            }
            LoopNum++;
            telemetry.addData("loopnum",LoopNum);
            telemetry.addData("heading",heading);
            rotationCorrection = -0.02*(heading);
            if (rotationCorrection>0.5){
                rotationCorrection=0.5;
            }
            if (rotationCorrection<-0.5){
                rotationCorrection=-0.5;
            }
            leftpower = (yLocation - 29)*-0.3+rotationCorrection;
            rightpower = (yLocation - 29)*0.3+rotationCorrection;
            if (Alliance==Blue){
                toppower = (xLocation - 50)*-0.3+rotationCorrection;
                bottompower = (xLocation - 50)*0.3+rotationCorrection;
            }            
            if (Alliance==Red){
                toppower = (xLocation - 50)*0.3+rotationCorrection;
                bottompower = (xLocation - 50)*-0.3+rotationCorrection;
            }
            if (Math.abs(leftpower) > 1.0){
                rightpower /= Math.abs(leftpower);
                toppower /= Math.abs(leftpower);
                bottompower /= Math.abs(leftpower);
                leftpower /= Math.abs(leftpower);
            }
            if (Math.abs(rightpower) > 1.0){
                leftpower /= Math.abs(rightpower);
                toppower /= Math.abs(rightpower);
                bottompower /= Math.abs(rightpower);
                rightpower /= Math.abs(rightpower);
            }
            if (Math.abs(toppower) > 1.0){
                rightpower /= Math.abs(toppower);
                leftpower /= Math.abs(toppower);
                bottompower /= Math.abs(toppower);
                toppower /= Math.abs(toppower);
            }
            if (Math.abs(bottompower) > 1.0){
                rightpower /= Math.abs(bottompower);
                leftpower /= Math.abs(bottompower);
                toppower /= Math.abs(bottompower);
                bottompower /= Math.abs(bottompower);
            }
            robot.leftDrive.setPower(leftpower);
            robot.rightDrive.setPower(rightpower);
            robot.TopDrive.setPower(toppower);
            robot.BottomDrive.setPower(bottompower);
            LocationEncoderR1 = robot.rightDrive.getCurrentPosition();
            LocationEncoderL1= robot.leftDrive.getCurrentPosition();
            LocationEncoderT1 = robot.TopDrive.getCurrentPosition();
            LocationEncoderB1 = robot.BottomDrive.getCurrentPosition();
            yLocation -= (LocationEncoderR1 - LocationEncoderR - LocationEncoderL1 + LocationEncoderL) 
                        / 2.0 / COUNTS_PER_INCH * Math.cos(heading*pi/180.0); 
            yLocation -= (LocationEncoderB1 - LocationEncoderB - LocationEncoderT1 + LocationEncoderT) 
                        / 2.0 / COUNTS_PER_INCH * Math.sin(heading*pi/180.0); 
            if(Alliance == Blue){
                xLocation += (LocationEncoderR1 - LocationEncoderR - LocationEncoderL1 + LocationEncoderL) 
                        / 2.0 / COUNTS_PER_INCH * Math.sin(heading*pi/180.0); 
                xLocation -= (LocationEncoderB1 - LocationEncoderB - LocationEncoderT1 + LocationEncoderT) 
                        / 2.0 / COUNTS_PER_INCH * Math.cos(heading*pi/180.0);
            }
            if(Alliance == Red){
                xLocation -= (LocationEncoderR1 - LocationEncoderR - LocationEncoderL1 + LocationEncoderL) 
                        / 2.0 / COUNTS_PER_INCH * Math.sin(heading*pi/180.0); 
                xLocation += (LocationEncoderB1 - LocationEncoderB - LocationEncoderT1 + LocationEncoderT) 
                        / 2.0 / COUNTS_PER_INCH * Math.cos(heading*pi/180.0);
            }
            LocationEncoderR=LocationEncoderR1;
            LocationEncoderL=LocationEncoderL1;
            LocationEncoderT=LocationEncoderT1;
            LocationEncoderB=LocationEncoderB1;
            telemetry.addData("x/y", "%.1f / %.1f", xLocation, yLocation);
            telemetry.update();
        }
        
        // Lower Pullers for foundation
        for(i = 1; i <= 100; i ++){
            robot.FoundPull.setPosition(1 + (robot.FoundPullPos-1)*i/100);
            robot.FoundPull2.setPosition(0 + (robot.FoundPull2Pos-0)*i/100);
            sleep(1);
        }
        EncoderPositionL = robot.leftDrive.getCurrentPosition() + 1 * COUNTS_PER_INCH;
        EncoderPositionR = robot.rightDrive.getCurrentPosition() - 1 * COUNTS_PER_INCH;
        while (!isStopRequested() && opModeIsActive()){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (robot.leftDrive.getCurrentPosition() > EncoderPositionL && 
                        robot.rightDrive.getCurrentPosition() < EncoderPositionR){
                break;       
            }
            heading=angles.firstAngle;
            LoopNum++;
            telemetry.addData("loopnum",LoopNum);
            telemetry.addData("heading",heading);

            speed = -0.02*(heading - 0);

            if (speed>0.3){
                speed=0.3;
            }
            if (speed<-0.3){
                speed=-0.3;
            }
            robot.TopDrive.setPower(speed);
            robot.BottomDrive.setPower(speed);
            robot.leftDrive.setPower(0.3+speed);
            robot.rightDrive.setPower(-0.3+speed);
            telemetry.addData("rotation",speed);
            telemetry.update();
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0); 
        robot.BottomDrive.setPower(0);
        robot.TopDrive.setPower(0);
        robot.FoundPull.setPosition(robot.FoundPullPos);
        robot.FoundPull2.setPosition(robot.FoundPull2Pos);
        sleep(500);
        runtime.reset();
        
        //pull foundation back, backing up at full speed, bring foundation to zone 
        EncoderPositionL = robot.leftDrive.getCurrentPosition();
        while (!isStopRequested() && opModeIsActive() && (runtime.seconds() < 15)){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading= angles.firstAngle;
            LoopNum++;
            telemetry.addData("loopnum",LoopNum);
            telemetry.addData("heading",heading);
            if (Alliance == Red){
                speed = -0.02*(heading - 0);
            }
            if (Alliance == Blue){
                speed = -0.02*(heading + 0);
            }
            if (speed>0.8){
                speed=0.8;
            }
            if (speed<-0.8){
                speed=-0.8;
            }
            robot.leftDrive.setPower(0.8 + speed);
            robot.rightDrive.setPower(-0.8 + speed);
            robot.BottomDrive.setPower(speed);
            robot.TopDrive.setPower(speed);
            telemetry.addData("speed",speed);
            telemetry.update();            
            if (robot.leftDrive.getCurrentPosition()-EncoderPositionL>34*COUNTS_PER_INCH){
                break;
            }
        }            
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        robot.TopDrive.setPower(0);
        sleep(100);
        
        //pullers up
        robot.FoundPull.setPosition(1);
        robot.FoundPull2.setPosition(0);
        
        EncoderPositionT = robot.TopDrive.getCurrentPosition();
        EncoderPositionB = robot.BottomDrive.getCurrentPosition();
        while (opmodeRunTime.seconds()<25){ 
            sleep(10);
        }

        //Drive to line to park
        EncoderPositionT = robot.TopDrive.getCurrentPosition();
        while (!isStopRequested() && opModeIsActive()){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            LoopNum++;
            heading = angles.firstAngle;
            telemetry.addData("Cos pi", Math.cos(0));
            telemetry.addData("loopnum",LoopNum);
            if (Alliance == Blue){
                leftpower = -Math.cos((angles.firstAngle+85)* pi /180);
                rightpower = Math.cos((angles.firstAngle+85)* pi /180);
                toppower = -Math.sin((angles.firstAngle+85)* pi /180);
                bottompower = Math.sin((angles.firstAngle+85)* pi /180);
            }
            if (Alliance == Red){
                leftpower = -Math.cos((angles.firstAngle-95)* pi /180);
                rightpower = Math.cos((angles.firstAngle-95)* pi /180);
                toppower = -Math.sin((angles.firstAngle-95)* pi /180);
                bottompower = Math.sin((angles.firstAngle-95)* pi /180);
            }
            if (Alliance == Blue){
                speed = -0.02*(heading - 10);
            }
            if (Alliance == Red){
                speed = -0.02*(heading + 10);
            }
            if (speed>0.3){
                speed=0.3;
            }
            if (speed<-0.3){
                speed=-0.3;
            }
            robot.leftDrive.setPower(leftpower + speed);
            robot.rightDrive.setPower(rightpower + speed);
            robot.BottomDrive.setPower(bottompower + speed);
            robot.TopDrive.setPower(toppower + speed);
            if ( Math.abs(robot.TopDrive.getCurrentPosition() - EncoderPositionT) > 40 * COUNTS_PER_INCH){
                break;
            }

            telemetry.addData("left speed", leftpower);
            telemetry.addData("heading", angles.firstAngle);
            telemetry.update();
        }
        
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0); 
        robot.BottomDrive.setPower(0);
        robot.TopDrive.setPower(0);
        robot.Light.setPower(0);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDriveForwards(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
                telemetry.addData("time", runtime.seconds());
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftDrive.getCurrentPosition(),
                                            robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            telemetry.addData("End", "Reached");

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveSideways(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newTopTarget;
        int newBottomTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTopTarget = robot.TopDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBottomTarget = robot.BottomDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.TopDrive.setTargetPosition(newTopTarget);
            robot.BottomDrive.setTargetPosition(newBottomTarget);

            // Turn On RUN_TO_POSITION
            robot.TopDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BottomDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.TopDrive.setPower(Math.abs(speed));
            robot.BottomDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.TopDrive.isBusy() && robot.BottomDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newTopTarget,  newBottomTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.TopDrive.getCurrentPosition(),
                                            robot.BottomDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.TopDrive.setPower(0);
            robot.BottomDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.TopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
