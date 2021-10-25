package org.firstinspires.ftc.teamcode1920;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.FileWriter;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

import java.util.Locale;

@Autonomous(name="BaseAuto", group ="Base")

public class MainBaseAuto extends LinearOpMode{
    RevRobotHardwareNew         robot   = new RevRobotHardwareNew();   
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime CenterRunTime = new ElapsedTime();
    float     oruntime;
    
    static final double     COUNTS_PER_MOTOR_REV    = 1300 ;    
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    double                  x;
    double                  y;
    double                  xLocation = -23; 
    double                  yLocation = 62;
    double                  LocationEncoderT;
    double                  LocationEncoderB;
    double                  LocationEncoderL;
    double                  LocationEncoderR;
    double                  LocationEncoderT1;
    double                  LocationEncoderB1;
    double                  LocationEncoderL1;
    double                  LocationEncoderR1;
    int                     ArmPosition;
    int                     LoopNum = 0;
    double                  heading;
    double                  speed;
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
    boolean                 Red = false;
    boolean                 Blue = true;
    boolean                 Alliance;
    boolean                 WriteLog = true;
    String                  FName = "/storage/emulated/0/download/testout.txt";
    double                  xTarget;
    double                  yTarget;
    double                  FoundPullPos;
    double                  FoundPull2Pos;
    int                     i;
    double                  pi = Math.acos(-1);
    boolean                 Stone;
    double                  ArmTarget;
    double                  ArmSpeed;
    double                  ArmDuration;
    double                  xDriveTarget;
    double                  yDriveTarget;
    double                  headingTarget; 
    double                  InBox;
    
    private static final String VUFORIA_KEY =
            "AR6S3Bf/////AAABmSiR88jls04NgdZrjkHGUR8qCg+SYQa4yPzwOZ0Fq/3uo94PIcP+Ux9YVfyqoRiVCy9BVdl7Lvpsvciw+4ZxtxmpR8GLeOM5iy2doyunkiljYeGJ3HKiZiqpYVJMRx18wxNZq2dzow1yMpayMEaUBcJFmDa7M3jSstzzCkP+8XOgEVNDNNw2gMbYabwNn/P8+bxbwCdPxVLPDlY2CQ5qkarTMCKOMfKZbxHdQnYKefZVkgUHEVzLU+uU+0i2tnJX19XCQPTev2nhxyOBjBX7ie23vYo7usaTD6kgmFpmBa3AniYHJW28V9c07NGSnZftHtTlVkRtBBY9o8i5ZkoMW4MJ0s0keSIKHI9Kinhq/lyd";
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final float stoneZ = 2.00f * mmPerInch;

    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    
    @Override public void runOpMode() {
        robot.init(hardwareMap);
        if(WriteLog){
            try{
                FileWriter fw=new FileWriter(FName);    
                fw.write("Start Log \n"); 
                fw.close();
            }catch(Exception e){System.out.println(e);}
        }
        BNO055IMU.Parameters Gyroparameters = new BNO055IMU.Parameters();
        Gyroparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Gyroparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Gyroparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        Gyroparameters.loggingEnabled      = true;
        Gyroparameters.loggingTag          = "IMU";
        Gyroparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;
        ElapsedTime opmodeRunTime = new ElapsedTime();
        ElapsedTime CenterRunTime = new ElapsedTime();

        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        telemetry.log().setCapacity(6);

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

         while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }
        opmodeRunTime.reset();
        int loopCount = 1;
        
        long start = System.currentTimeMillis();
        phoneYRotate = 0;

        final float CAMERA_FORWARD_DISPLACEMENT  = 0 * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 0 * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT     = 0 * mmPerInch;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        composeTelemetry();

        waitForStart();
        
        opmodeRunTime.reset(); 
        if(WriteLog){
            try{
                FileWriter fw=new FileWriter(FName, true);    
                fw.write("Start pressed. Started timer \nTime: " + opmodeRunTime.seconds() + " seconds\n"); 
                fw.close();
            }catch(Exception e){System.out.println(e);}
        }
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(Gyroparameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
        telemetry.addData("Alliance", Alliance);
        telemetry.update();
        
        //get servos / lights in place
        robot.Claw.setPosition(0.35);
        robot.Light.setPower(0.5);
        robot.FoundPull.setPosition(0.6);
        robot.FoundPull2.setPosition(0);
        robot.Capstone.setPosition(0.0);
        sleep(100);
        
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.TopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        LocationEncoderR = robot.rightDrive.getCurrentPosition();
        LocationEncoderL = robot.leftDrive.getCurrentPosition();
        LocationEncoderT = robot.TopDrive.getCurrentPosition();
        LocationEncoderB = robot.BottomDrive.getCurrentPosition();
        
        // Drive forward to position where we can see Skystone if in position 1 or 2. 
        EncoderPositionR = robot.rightDrive.getCurrentPosition();
        speed = 0;
        runtime.reset();
        while (!isStopRequested() && opModeIsActive()){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading=angles.firstAngle;
            if (Math.abs(yLocation) > 42.5 && Math.abs(yLocation) < 43.5 && 
                        Math.abs(xLocation) > 27.5 && Math.abs(xLocation) < 28.5 && heading < 3 && heading > -3){
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.TopDrive.setPower(0);
                robot.BottomDrive.setPower(0);
                break;          
            }
            LoopNum++;
            speed = runtime.seconds() * robot.maxacceleration;
            if (speed > 1){
                speed = 1;
            }
            telemetry.addData("loopnum",LoopNum);
            telemetry.addData("heading",heading);
            rotationCorrection = -0.02*(heading);
            if (rotationCorrection>0.5){
                rotationCorrection=0.5;
            }
            if (rotationCorrection<-0.5){
                rotationCorrection=-0.5;
            }
            leftpower = (yLocation - 43)*-0.3+rotationCorrection;
            rightpower = (yLocation - 43)*0.3+rotationCorrection;
            if (Alliance==Blue){
                toppower = (xLocation + 28)*-0.3+rotationCorrection;
                bottompower = (xLocation + 28)*0.3+rotationCorrection;
            }            
            if (Alliance==Red){
                toppower = (xLocation + 28)*0.3+rotationCorrection;
                bottompower = (xLocation + 28)*-0.3+rotationCorrection;
            }
            if (Math.abs(leftpower) > speed){
                rightpower /= Math.abs(leftpower)/speed;
                toppower /= Math.abs(leftpower)/speed;
                bottompower /= Math.abs(leftpower)/speed;
                leftpower /= Math.abs(leftpower)/speed;
            }
            if (Math.abs(rightpower) > speed){
                leftpower /= Math.abs(rightpower)/speed;
                toppower /= Math.abs(rightpower)/speed;
                bottompower /= Math.abs(rightpower)/speed;
                rightpower /= Math.abs(rightpower)/speed;
            }
            if (Math.abs(toppower) > speed){
                rightpower /= Math.abs(toppower)/speed;
                leftpower /= Math.abs(toppower)/speed;
                bottompower /= Math.abs(toppower)/speed;
                toppower /= Math.abs(toppower)/speed;
            }
            if (Math.abs(bottompower) > speed){
                rightpower /= Math.abs(bottompower)/speed;
                leftpower /= Math.abs(bottompower)/speed;
                toppower /= Math.abs(bottompower)/speed;
                bottompower /= Math.abs(bottompower)/speed;
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
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.TopDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        if(WriteLog){
            try{
                FileWriter fw=new FileWriter(FName, true);    
                fw.write("Driven Forward. \nTime: " + opmodeRunTime.seconds() + " seconds\n"); 
                fw.write("xLocation / yLocation = " + xLocation + " / " + yLocation + "\n"); 
                fw.close();
            }catch(Exception e){System.out.println(e);}
        }
        targetsSkyStone.activate();
        telemetry.addData("x/y", "%.1f / %.1f", xLocation, yLocation);
        telemetry.update();  
        if(WriteLog){
            try{
                FileWriter fw=new FileWriter(FName, true);    
                fw.write("After sleep. \nTime: " + opmodeRunTime.seconds() + " seconds\n"); 
                fw.write("xLocation / yLocation = " + xLocation + " / " + yLocation + "\n"); 
                fw.close();
            }catch(Exception e){System.out.println(e);}
        }
        runtime.reset();
        
        //look for stone
        Stone = false;
        LoopNum = 0;
        while (!isStopRequested() && opModeIsActive()) {
            LoopNum += 1;
            telemetry.addData("LoopNum", LoopNum);
            telemetry.update();
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            if (targetVisible) {
                // express position (translation) of robot in inches.
                Stone = true;
                robot.TopDrive.setPower(0);
                robot.BottomDrive.setPower(0);
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                telemetry.update();
                if (translation.get(1) / mmPerInch > -5.5 && translation.get(1) / mmPerInch < 11.5){
                    break;
                } else {
                    targetVisible = false;
                }
            }
            if(targetVisible == false) { 
                if(LoopNum < 10){
                    sleep(100);
                    telemetry.addData("LoopNum", LoopNum);
                    telemetry.update();
                    continue;
                }
                if(LoopNum > 11 && LoopNum < 20){
                    sleep(100);
                    telemetry.addData("LoopNum", LoopNum);
                    telemetry.update();
                    continue;
                }
                if(LoopNum >= 20){
                    telemetry.addData("Visible Target", "none, giving up");
                    telemetry.update();
                    encoderDriveForward(DRIVE_SPEED, -10, 10, 3);
                    ArmPosition = robot.Arm.getCurrentPosition();
                    while (robot.Arm.getCurrentPosition()-ArmPosition < 100 ){
                        telemetry.addData("arm position",robot.Arm.getCurrentPosition()-ArmPosition);
                        telemetry.update();
                        robot.Arm.setPower(0.4);
                        robot.Arm2.setPower(0.4);
                    }
                    robot.Arm.setPower(0);
                    robot.Arm2.setPower(0);
                    
                    //robot drives forwards
                    encoderDriveForward(DRIVE_SPEED, -5, 5, 0.2);
                    
                    for(i = 1; i <= 100; i ++){
                        robot.Claw.setPosition(1 + (1.0-1)*i/100);
                        sleep(10);
                    }
                    while (robot.Arm.getCurrentPosition()>ArmPosition + 10){
                        telemetry.addData("arm position",robot.Arm.getCurrentPosition()-ArmPosition);
                        telemetry.update();
                        robot.Arm.setPower(-0.5);
                        robot.Arm2.setPower(-0.5);
                    }
                    break;
                }
                telemetry.addData("LoopNum", LoopNum);
                telemetry.addData("Visible Target", "none");
                telemetry.addData("Start", "Driving");
                telemetry.update();
                robot.TopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.BottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 
                if(WriteLog){
                    try{
                        FileWriter fw=new FileWriter(FName, true);    
                        fw.write("Skystone not Detected. \n Driving sideways...\nTime: " + opmodeRunTime.seconds() + " seconds\n"); 
                        fw.close();
                    }catch(Exception e){System.out.println(e);
                }
            }
            //drive to position to see stones 3 and 4
            xDriveTarget = 39;
            yDriveTarget = 43;
            speed = 0;
            runtime.reset();
            while(Math.abs(yLocation) < yDriveTarget-0.5 || Math.abs(yLocation) > yDriveTarget+0.5 || 
                    (Math.abs(xLocation) < xDriveTarget-0.5) || (Math.abs(xLocation) > xDriveTarget+0.5)){
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
                speed = runtime.seconds() * robot.maxacceleration;
                telemetry.addData("heading",heading);
                rotationCorrection = -0.02*(heading);
                if (rotationCorrection>0.5){
                    rotationCorrection=0.5;
                }
                if (rotationCorrection<-0.5){
                    rotationCorrection=-0.5;
                }
                leftpower = (yLocation - yDriveTarget)*-0.3+rotationCorrection;
                rightpower = (yLocation - yDriveTarget)*0.3+rotationCorrection;
                if (Alliance==Blue){
                    toppower = (xLocation + xDriveTarget)*-0.3+rotationCorrection;
                    bottompower = (xLocation + xDriveTarget)*0.3+rotationCorrection;
                }            
                if (Alliance==Red){
                    toppower = (xLocation + xDriveTarget)*0.3+rotationCorrection; 
                    bottompower = (xLocation + xDriveTarget)*-0.3+rotationCorrection;
                }
                if (Math.abs(leftpower) > speed){
                rightpower /= Math.abs(leftpower)/speed;
                toppower /= Math.abs(leftpower)/speed;
                bottompower /= Math.abs(leftpower)/speed;
                leftpower /= Math.abs(leftpower)/speed;
                }
                if (Math.abs(rightpower) > speed){
                    leftpower /= Math.abs(rightpower)/speed;
                    toppower /= Math.abs(rightpower)/speed;
                    bottompower /= Math.abs(rightpower)/speed;
                    rightpower /= Math.abs(rightpower)/speed;
                }
                if (Math.abs(toppower) > speed){
                    rightpower /= Math.abs(toppower)/speed;
                    leftpower /= Math.abs(toppower)/speed;
                    bottompower /= Math.abs(toppower)/speed;
                    toppower /= Math.abs(toppower)/speed;
                }
                if (Math.abs(bottompower) > speed){
                    rightpower /= Math.abs(bottompower)/speed;
                    leftpower /= Math.abs(bottompower)/speed;
                    toppower /= Math.abs(bottompower)/speed;
                    bottompower /= Math.abs(bottompower)/speed;
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
            }
            robot.TopDrive.setPower(0);
            robot.BottomDrive.setPower(0);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            sleep(50);
            telemetry.update();
            }
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.TopDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        
        // Skystone has possibly been detected
        if(WriteLog && Stone){
            try{
                FileWriter fw=new FileWriter(FName, true);    
                fw.write("Skystone Detected. \nTime: " + opmodeRunTime.seconds() + " seconds\n"); 
                fw.close();
            }catch(Exception e){System.out.println(e);}
        }
        //updating x and y, maintaining heading if skystone was found
        telemetry.addData("We've", "Gotten through the loop");
        telemetry.update();
        LoopNum = 0;
        xTarget = -9;
        yTarget = 7.7;
        CenterRunTime.reset();
        while (!isStopRequested() && opModeIsActive() && Stone == true) {
            telemetry.addData("loop", "x/y");
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            if (targetVisible) {
                LoopNum += 1;
                telemetry.addData("Loop Number", LoopNum);
                telemetry.update();
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading=angles.firstAngle;
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                        
                x = translation.get(0)/mmPerInch;
                y = translation.get(1)/mmPerInch;
            
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                
                //start driving to match x and y
                robot.TopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.BottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                TPower = (y- yTarget)*-0.2;
                BPower = (y - yTarget)*0.2;
                LPower = (x - xTarget)*0.2;
                RPower = (x - xTarget)*-0.2;
                if (TPower>0.3){
                    TPower=0.3;
                }
                if (TPower<-0.3){
                    TPower=-0.3;
                }
                if (BPower>0.3){
                    BPower=0.3;
                }
                if (BPower<-0.3){
                    BPower=-0.3;
                }
                if (LPower>0.3){
                    LPower=0.3;
                }
                if (LPower<-0.3){
                    LPower=-0.3;
                }
                if (RPower>0.3){
                    RPower=0.3;
                }
                if (RPower<-0.3){
                    RPower=-0.3;
                }
                speed = -0.01*(heading);
                if (speed>0.3){
                    speed=0.3;
                }
                if (speed<-0.3){
                    speed=-0.3;
                }
                LPower += speed;
                RPower += speed;
                BPower += speed;
                TPower += speed;
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
                telemetry.addData("speed", speed);
                telemetry.addData("LPower", LPower);
                telemetry.addData("RPower", RPower);
                telemetry.addData("BPower", BPower);
                telemetry.addData("TPower", TPower);
                robot.leftDrive.setPower(LPower);
                robot.rightDrive.setPower(RPower);
                robot.BottomDrive.setPower(BPower);
                robot.TopDrive.setPower(TPower);
                if (y < yTarget + 0.5 && y > yTarget - 0.5
                            && x < xTarget + 0.5 && x > xTarget - 0.5
                            && heading < 3 && heading > -3){
                    if(WriteLog){
                        try{
                            FileWriter fw=new FileWriter(FName, true);    
                            fw.write(String.format("x; %.1f \n", translation.get(0) / mmPerInch));  
                            fw.write(String.format("y; %.1f \n", translation.get(1) / mmPerInch));
                            fw.write(String.format("heading; %.1f \n", angles.firstAngle));
                            fw.close();
                        }catch(Exception e){System.out.println(e);}
                    }
                    if (CenterRunTime.seconds() > 1){
                        break;
                    }
                }else {
                    CenterRunTime.reset();
                }
            }
            else {
                telemetry.addData("Visible Target", "none");
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.BottomDrive.setPower(0);
                robot.TopDrive.setPower(0);
            }
            telemetry.update();
        }
        telemetry.addData("Out of", "First Loop");
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        robot.TopDrive.setPower(0);
        if(WriteLog){
            try{
                FileWriter fw=new FileWriter(FName, true);    
                fw.write(String.format("Before Arm \n"));
                fw.close();
            }catch(Exception e){System.out.println(e);}
        }
        //putting arm down to pick up the first skystone
        if(Stone == true){
            runtime.reset();
            ArmDuration = 1;
            while(runtime.seconds()<= ArmDuration){
                ArmTarget = runtime.seconds()*(100/ArmDuration);
                if(ArmTarget > 100){
                    ArmTarget = 100;
                }
                if(WriteLog){
                    try{
                        FileWriter fw=new FileWriter(FName, true);    
                        fw.write("ArmTarget;" + ArmTarget + "\n"); 
                        fw.write("Runtime Seconds;" + runtime.seconds() + "\n"); 
                        fw.write("ArmSpeed;" + ArmSpeed + "\n"); 
                        fw.write("Arm Current Position;" + robot.Arm.getCurrentPosition() + "\n"); 
                        fw.close();
                    }catch(Exception e){System.out.println(e);}
                }
                ArmSpeed = -0.02*(robot.Arm.getCurrentPosition()-ArmTarget);
                if(ArmSpeed > 1){
                    ArmSpeed = 1;
                }
                robot.Arm.setPower(ArmSpeed);
                robot.Arm2.setPower(ArmSpeed);
            }
            robot.Arm.setPower(0);
            robot.Arm2.setPower(0);
            if(WriteLog){
                try{
                    FileWriter fw=new FileWriter(FName, true);    
                    fw.write(String.format("Before Claw; %.1f \n", ArmTarget)); 
                    fw.close();
                }catch(Exception e){System.out.println(e);}
            }
            encoderDriveForward(DRIVE_SPEED, -5, 5, 0.2);
            for(i = 1; i <= 100; i ++){
                robot.Claw.setPosition(1 + (1.0-1)*i/100);
                sleep(10);
            }
            runtime.reset();
            while(runtime.seconds() < ArmDuration){
                ArmTarget = 100 - runtime.seconds()*(100/ArmDuration);
                if(ArmTarget < 0){
                    ArmTarget = 0;
                }
                ArmSpeed = -0.02*(robot.Arm.getCurrentPosition()-ArmTarget);
                robot.Arm.setPower(ArmSpeed);
                robot.Arm2.setPower(ArmSpeed);
            }
            robot.Arm.setPower(0);
            robot.Arm2.setPower(0);
        }
        robot.FoundPull.setPosition(1.0);
        xDriveTarget = 43;
        yDriveTarget = 42;
        speed = 0;
        runtime.reset();
        //drive to building side
        while (!isStopRequested() && opModeIsActive()){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading=angles.firstAngle;
            speed = runtime.seconds() * robot.maxacceleration;
            if (Math.abs(yLocation) > yDriveTarget - 0.5 && Math.abs(yLocation) < yDriveTarget + 0.5 && 
                        xLocation > xDriveTarget - 0.5 && xLocation < xDriveTarget + 0.5 && heading < 3 && heading > -3){
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
            if (Alliance==Blue){
                toppower = (xLocation - xDriveTarget)*-0.3;
                bottompower = (xLocation - xDriveTarget)*0.3;
            }            
            if (Alliance==Red){
                toppower = (xLocation - xDriveTarget)*0.3;
                bottompower = (xLocation - xDriveTarget)*-0.3;
            }
            if(Math.abs(toppower) > 1.0){
                toppower /= Math.abs(toppower);
                bottompower /= Math.abs(bottompower);
            }
            toppower += rotationCorrection;
            bottompower += rotationCorrection;
            leftpower = (yLocation - yDriveTarget)*-0.3+rotationCorrection;
            rightpower = (yLocation - yDriveTarget)*0.3+rotationCorrection;
            if (Math.abs(leftpower) > speed){
                rightpower /= Math.abs(leftpower)/speed;
                toppower /= Math.abs(leftpower)/speed;
                bottompower /= Math.abs(leftpower)/speed;
                leftpower /= Math.abs(leftpower)/speed;
            }
            if (Math.abs(rightpower) > speed){
                leftpower /= Math.abs(rightpower)/speed;
                toppower /= Math.abs(rightpower)/speed;
                bottompower /= Math.abs(rightpower)/speed;
                rightpower /= Math.abs(rightpower)/speed;
            }
            if (Math.abs(toppower) > speed){
                rightpower /= Math.abs(toppower)/speed;
                leftpower /= Math.abs(toppower)/speed;
                bottompower /= Math.abs(toppower)/speed;
                toppower /= Math.abs(toppower)/speed;
            }
            if (Math.abs(bottompower) > speed){
                rightpower /= Math.abs(bottompower)/speed;
                leftpower /= Math.abs(bottompower)/speed;
                toppower /= Math.abs(bottompower)/speed;
                bottompower /= Math.abs(bottompower)/speed;
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
        //drive forward towards foundation
        xDriveTarget = 43;
        yDriveTarget = 30;
        speed = 0;
        runtime.reset();
        while (!isStopRequested() && opModeIsActive()){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading=angles.firstAngle;
            speed = runtime.seconds() * robot.maxacceleration;
            if (Math.abs(yLocation) > yDriveTarget - 0.5 && Math.abs(yLocation) < yDriveTarget + 0.5 && 
                        xLocation > xDriveTarget - 0.5 && xLocation < xDriveTarget + 0.5 && heading < 3 && heading > -3){
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
            if (Alliance==Blue){
                toppower = (xLocation - xDriveTarget)*-0.3;
                bottompower = (xLocation - xDriveTarget)*0.3;
            }            
            if (Alliance==Red){
                toppower = (xLocation - xDriveTarget)*0.3;
                bottompower = (xLocation - xDriveTarget)*-0.3;
            }
            if(Math.abs(toppower) > 1.0){
                toppower /= Math.abs(toppower);
                bottompower /= Math.abs(bottompower);
            }
            toppower += rotationCorrection;
            bottompower += rotationCorrection;
            leftpower = (yLocation - yDriveTarget)*-0.3+rotationCorrection;
            rightpower = (yLocation - yDriveTarget)*0.3+rotationCorrection;
            if (Math.abs(leftpower) > speed){
                rightpower /= Math.abs(leftpower)/speed;
                toppower /= Math.abs(leftpower)/speed;
                bottompower /= Math.abs(leftpower)/speed;
                leftpower /= Math.abs(leftpower)/speed;
            }
            if (Math.abs(rightpower) > speed){
                leftpower /= Math.abs(rightpower)/speed;
                toppower /= Math.abs(rightpower)/speed;
                bottompower /= Math.abs(rightpower)/speed;
                rightpower /= Math.abs(rightpower)/speed;
            }
            if (Math.abs(toppower) > speed){
                rightpower /= Math.abs(toppower)/speed;
                leftpower /= Math.abs(toppower)/speed;
                bottompower /= Math.abs(toppower)/speed;
                toppower /= Math.abs(toppower)/speed;
            }
            if (Math.abs(bottompower) > speed){
                rightpower /= Math.abs(bottompower)/speed;
                leftpower /= Math.abs(bottompower)/speed;
                toppower /= Math.abs(bottompower)/speed;
                bottompower /= Math.abs(bottompower)/speed;
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
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0); 
        robot.BottomDrive.setPower(0);
        robot.TopDrive.setPower(0);
                
        // Lower Pullers for foundation
        for(i = 1; i <= 100; i ++){
            robot.FoundPull.setPosition(1 + (robot.FoundPullPos-1)*i/100);
            robot.FoundPull2.setPosition(0 + (robot.FoundPull2Pos-0)*i/100);
            sleep(1);
        }
        
        //Back up one inch 
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
            ArmDuration = 1;
            if(runtime.seconds()<= ArmDuration){
                ArmTarget = runtime.seconds()*(100/ArmDuration);
                if(ArmTarget > 100){
                    ArmTarget = 100;
                }
                ArmSpeed = -0.02*(robot.Arm.getCurrentPosition()-ArmTarget);
                robot.Arm.setPower(ArmSpeed);
                robot.Arm2.setPower(ArmSpeed);
            }
            else if(runtime.seconds() > ArmDuration){
                robot.Claw.setPosition(0);
            }
            if(runtime.seconds() > (ArmDuration + 0.1) && runtime.seconds() <= (ArmDuration+0.1) + ArmDuration){
                ArmTarget = 100 - (runtime.seconds()-(ArmDuration+0.1))*(100/ArmDuration);
                if(ArmTarget < 0){
                    ArmTarget = 0;
                }
                ArmSpeed = -0.02*(robot.Arm.getCurrentPosition()-ArmTarget);
                robot.Arm.setPower(ArmSpeed);
                robot.Arm2.setPower(ArmSpeed);
            }
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading= angles.firstAngle;
            LoopNum++;
            telemetry.addData("loopnum",LoopNum);
            telemetry.addData("heading",heading);
            if (Alliance == Red){
                speed = -0.02*(heading - 5);
            }
            if (Alliance == Blue){
                speed = -0.02*(heading + 5);
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
            if (robot.leftDrive.getCurrentPosition()-EncoderPositionL>37*COUNTS_PER_INCH){
                break;
            }
        }
        sleep(10);
        //pullers up
        robot.FoundPull.setPosition(1);
        robot.FoundPull2.setPosition(0);
        sleep(500);
        encoderDriveForward(DRIVE_SPEED, -1, 1, 0.2);
        //go past foundation
        EncoderPositionT = robot.TopDrive.getCurrentPosition();
        while (opModeIsActive()){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            LoopNum++;
            heading = angles.firstAngle;
            telemetry.addData("Cos pi", Math.cos(0));
            telemetry.addData("loopnum",LoopNum);
            if (Alliance == Blue){
                leftpower = -Math.cos((angles.firstAngle+80)* pi /180);
                rightpower = Math.cos((angles.firstAngle+80)* pi /180);
                toppower = -Math.sin((angles.firstAngle+80)* pi /180);
                bottompower = Math.sin((angles.firstAngle+80)* pi /180);
            }
            if (Alliance == Red){
                leftpower = -Math.cos((angles.firstAngle-100)* pi /180);
                rightpower = Math.cos((angles.firstAngle-100)* pi /180);
                toppower = -Math.sin((angles.firstAngle-100)* pi /180);
                bottompower = Math.sin((angles.firstAngle-100)* pi /180);
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
            if (Alliance == Blue && robot.TopDrive.getCurrentPosition() - EncoderPositionT < -30 * COUNTS_PER_INCH){
                break;
            }
            if (Alliance == Red && robot.TopDrive.getCurrentPosition() - EncoderPositionT > 30 * COUNTS_PER_INCH){
                break;
            }
            telemetry.addData("left speed", leftpower);
            telemetry.addData("heading", angles.firstAngle);
            telemetry.update();
        }
        runtime.reset();
        EncoderPositionL = robot.leftDrive.getCurrentPosition();
        
        //forward
        while (!isStopRequested() && opModeIsActive() && (runtime.seconds() < 15)){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading=angles.firstAngle;
            LoopNum++;
            telemetry.addData("loopnum",LoopNum);
            telemetry.addData("heading",heading);
            speed = -0.02*(heading);
            if (speed>0.8){
                speed=0.8;
            }
            if (speed<-0.8){
                speed=-0.8;
            }
            robot.leftDrive.setPower(-0.8 + speed);
            robot.rightDrive.setPower(0.8 + speed);
            robot.BottomDrive.setPower(speed);
            robot.TopDrive.setPower(speed);
            telemetry.addData("speed",speed);
            telemetry.update();            
            if (robot.leftDrive.getCurrentPosition()-EncoderPositionL<-15*COUNTS_PER_INCH){
                break;
            }
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        robot.TopDrive.setPower(0);
        
        //bumps the foundation further into the zone
        EncoderPositionT=robot.TopDrive.getCurrentPosition();
        while (!isStopRequested() && opModeIsActive() && (runtime.seconds() < 15)){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading=angles.firstAngle;
            LoopNum++;
            telemetry.addData("loopnum",LoopNum);
            telemetry.addData("heading",heading);
            rotationCorrection = -0.02*(heading);
            if (rotationCorrection>0.8){
                rotationCorrection=0.8;
            }
            if (rotationCorrection<-0.8){
                rotationCorrection=-0.8;
            }
            robot.leftDrive.setPower(0+ rotationCorrection);
            robot.rightDrive.setPower(0+ rotationCorrection);
            if (Alliance == Blue){
                robot.BottomDrive.setPower(-0.8 + rotationCorrection);
                robot.TopDrive.setPower(0.8 + rotationCorrection);
            }
            if (Alliance == Red){
                robot.BottomDrive.setPower(0.8 + rotationCorrection);
                robot.TopDrive.setPower(-0.8 + rotationCorrection);
            }
            telemetry.addData("RotationCorrection",rotationCorrection);
            telemetry.update();            
            if (Math.abs(robot.TopDrive.getCurrentPosition()-EncoderPositionT) > 13*COUNTS_PER_INCH){
                break;
            }
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        robot.TopDrive.setPower(0);
        
        EncoderPositionT = robot.TopDrive.getCurrentPosition();
        while (opModeIsActive()){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            LoopNum++;
            heading = angles.firstAngle;
            telemetry.addData("loopnum",LoopNum);
            rotationCorrection = -0.02*(heading);            
            if (rotationCorrection>0.3){
                rotationCorrection=0.3;
            }
            if (rotationCorrection<-0.3){
                rotationCorrection=-0.3;
            }
            if (Alliance == Blue){
                leftpower = -Math.cos((angles.firstAngle+70)* pi /180);
                rightpower = Math.cos((angles.firstAngle+70)* pi /180);
                toppower = -Math.sin((angles.firstAngle+70)* pi /180);
                bottompower = Math.sin((angles.firstAngle+70)* pi /180);
            }
            if (Alliance == Red){
                leftpower = -Math.cos((angles.firstAngle-70)* pi /180);
                rightpower = Math.cos((angles.firstAngle-70)* pi /180);
                toppower = -Math.sin((angles.firstAngle-70)* pi /180);
                bottompower = Math.sin((angles.firstAngle-70)* pi /180);
            }
            robot.leftDrive.setPower(leftpower + rotationCorrection);
            robot.rightDrive.setPower(rightpower + rotationCorrection);
            robot.BottomDrive.setPower(bottompower + rotationCorrection);
            robot.TopDrive.setPower(toppower + rotationCorrection);
            if (Math.abs(robot.TopDrive.getCurrentPosition() - EncoderPositionT) > 24 * COUNTS_PER_INCH){
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
        if(WriteLog){
            try{
                FileWriter fw=new FileWriter(FName, true);  
                if (isStopRequested()){
                    fw.write("Stop Requested.\n"); 
                }
                fw.write("End of Program. \nElapsed Time: " + opmodeRunTime.seconds() + " seconds.\n"); 
                fw.close();
            }catch(Exception e){System.out.println(e);}
        }
        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
        
    }
    public void encoderDriveForward(double speed,
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
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

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
    void composeTelemetry() {
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
            .addData("grvty", new Func<String>() {
                @Override public String value() {
                    return gravity.toString();
                    }
                })
            .addData("mag", new Func<String>() {
                @Override public String value() {
                    return String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel*gravity.xAccel
                                    + gravity.yAccel*gravity.yAccel
                                    + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
