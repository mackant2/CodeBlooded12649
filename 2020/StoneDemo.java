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

@Autonomous(name="Stone Demo", group ="Demo")

public class StoneDemo extends LinearOpMode{
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
        
        targetsSkyStone.activate();
        telemetry.addData("x/y", "%.1f / %.1f", xLocation, yLocation);
        telemetry.update();  
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
                    telemetry.addData("LoopNum", LoopNum);
                    telemetry.update();
                    continue;
                }
                if(LoopNum >= 20){
                    telemetry.addData("Visible Target", "none, giving up");
                    telemetry.update();
                    encoderDriveForward(DRIVE_SPEED, -16, 16, 0.2);
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
            sleep(500);
            telemetry.update();
            }
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.TopDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        // Skystone has possibly been detected
        //updating x and y, maintaining heading if skystone was found
        telemetry.addData("We've", "Gotten through the loop");
        telemetry.update();
        LoopNum = 0;
        CenterRunTime.reset();
        while (!isStopRequested() && opModeIsActive() && Stone == true) {
            telemetry.addData("loop", "x/y");
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                LoopNum += 1;
                telemetry.addData("Loop Number", LoopNum);
                telemetry.update();
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                //start driving to match x and y
                robot.TopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.BottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                TPower = ((translation.get(1) / mmPerInch) - 7.7)*-0.2;
                BPower = ((translation.get(1) / mmPerInch) - 7.7)*0.2;
                LPower = ((translation.get(0) / mmPerInch) + 8.25)*0.2;
                RPower = ((translation.get(0) / mmPerInch) + 8.25)*-0.2;
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
                speed = -0.01*(rotation.thirdAngle - 70);
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
                if(WriteLog){
                    try{
                        FileWriter fw=new FileWriter(FName, true);    
                        //fw.write(String.format("x; %.1f \n", translation.get(0) / mmPerInch));  
                        fw.write(String.format("y; %.1f \n", translation.get(1) / mmPerInch));
                        fw.write(String.format("toppower; %.1f \n", TPower));
                        fw.write(String.format("bottompower; %.1f \n", BPower));
                        fw.write(String.format("heading; %.1f \n", rotation.thirdAngle));
                        fw.close();
                    }catch(Exception e){System.out.println(e);}
                }
                robot.leftDrive.setPower(LPower);
                robot.rightDrive.setPower(RPower);
                robot.BottomDrive.setPower(BPower);
                robot.TopDrive.setPower(TPower);
                if ((translation.get(1) / mmPerInch) < 8.2 && (translation.get(1) / mmPerInch) > 7.2 
                            && (translation.get(0) / mmPerInch) > -8.75 && (translation.get(0) / mmPerInch) < -7.75
                            && rotation.thirdAngle < 72 && rotation.thirdAngle > 68){
                    if(WriteLog){
                        try{
                            FileWriter fw=new FileWriter(FName, true);    
                            fw.write(String.format("x; %.1f \n", translation.get(0) / mmPerInch));  
                            fw.write(String.format("y; %.1f \n", translation.get(1) / mmPerInch));
                            fw.write(String.format("heading; %.1f \n", rotation.thirdAngle));
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
