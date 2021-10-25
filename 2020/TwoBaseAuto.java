package org.firstinspires.ftc.teamcode1920;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.lang.annotation.Target;
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

@Autonomous(name="TwoBaseAuto", group ="Base")

public class TwoBaseAuto extends LinearOpMode{
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
    double                  sleep;
    double                  xLocation = -38.5; 
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
    double                  HeadingTarget;
    double                  HeadingTargetFinal;
    double                  FoundPullPos;
    double                  FoundPull2Pos;
    int                     i;
    double                  pi = Math.acos(-1);
    boolean                 Stone;
    int                     StonePos;
    double                  xStonePos;
    double                  bearing;
    double                  stoneXtarget;
    double                  stoneYtarget;
    double                  stoneHeading;
    
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
                fw.write("Starting TwoBaseAuto. \n");
                if (Alliance==Blue){ 
                    fw.write("Blue Alliance.\n");
                } else{
                    fw.write("Red Alliance.\n");
                }
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
        
        telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
        telemetry.addData("Alliance", Alliance);
        telemetry.update();
        if (!isStopRequested() && opModeIsActive()){
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(Gyroparameters);
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
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
        }
        // Drive to position where we can see Skystone if in position 5 or 6. 
        xTarget = -62.5;
        yTarget = 45;
        EncoderPositionR = robot.rightDrive.getCurrentPosition();
        runtime.reset();
        speed = 0;
        while (!isStopRequested() && opModeIsActive()){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading=angles.firstAngle;
            speed = runtime.seconds() * robot.maxacceleration;
            if (Math.abs(yLocation) > Math.abs(yTarget)-0.5 && Math.abs(yLocation) < Math.abs(yTarget)+0.5 && 
                        Math.abs(xLocation) > Math.abs(xTarget)-0.5 && Math.abs(xLocation) < Math.abs(xTarget)+0.5
                        && heading < 3 && heading > -3){
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.TopDrive.setPower(0);
                robot.BottomDrive.setPower(0);
                break;          
            }
            LoopNum++;
            telemetry.addData("loopnum5",LoopNum);
            telemetry.addData("heading",heading);
            rotationCorrection = -0.02*(heading);
            if (rotationCorrection>0.5){
                rotationCorrection=0.5;
            }
            if (rotationCorrection<-0.5){
                rotationCorrection=-0.5;
            }
            leftpower = (yLocation - yTarget)*-0.2+rotationCorrection;
            rightpower = (yLocation - yTarget)*0.2+rotationCorrection;
            if (Alliance==Blue){
                toppower = (xLocation - xTarget)*-0.2+rotationCorrection;
                bottompower = (xLocation - xTarget)*0.2+rotationCorrection;
            }            
            if (Alliance==Red){
                toppower = (xLocation - xTarget)*0.2+rotationCorrection;
                bottompower = (xLocation - xTarget)*-0.2+rotationCorrection;
            }
            if (Math.abs(leftpower) > 1.0){
                rightpower /= Math.abs(leftpower)/speed;
                toppower /= Math.abs(leftpower)/speed;
                bottompower /= Math.abs(leftpower)/speed;
                leftpower /= Math.abs(leftpower)/speed;
            }
            if (Math.abs(rightpower) > 1.0){
                leftpower /= Math.abs(rightpower)/speed;
                toppower /= Math.abs(rightpower)/speed;
                bottompower /= Math.abs(rightpower)/speed;
                rightpower /= Math.abs(rightpower)/speed;
            }
            if (Math.abs(toppower) > 1.0){
                rightpower /= Math.abs(toppower)/speed;
                leftpower /= Math.abs(toppower)/speed;
                bottompower /= Math.abs(toppower)/speed;
                toppower /= Math.abs(toppower)/speed;
            }
            if (Math.abs(bottompower) > 1.0){
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
            if (xLocation<-62.5){
                xLocation = -62.5;
            }
            LocationEncoderR=LocationEncoderR1;
            LocationEncoderL=LocationEncoderL1;
            LocationEncoderT=LocationEncoderT1;
            LocationEncoderB=LocationEncoderB1;
            telemetry.addData("x/y", "%.1f / %.1f", xLocation, yLocation);
            telemetry.update();
        }
        telemetry.addData("Out of", "First Loop");
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.TopDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        if(WriteLog){
            try{
                FileWriter fw=new FileWriter(FName, true);    
                fw.write("Driven to position to see stones 5 and 6. \nTime: " + opmodeRunTime.seconds() + " seconds\n"); 
                fw.write("xLocation / yLocation = " + xLocation + " / " + yLocation + "\n"); 
                fw.close();
            }catch(Exception e){System.out.println(e);}
        }
        if (!isStopRequested() && opModeIsActive()){
            targetsSkyStone.activate();
        }
        telemetry.addData("x/y", "%.1f / %.1f", xLocation, yLocation);
        telemetry.update();  
        
        runtime.reset();
        //Look for Skystone. Positions 5 and 6 should be visible initially.
        Stone = false;
        LoopNum = 0;
        while (!isStopRequested() && opModeIsActive()) {
            LoopNum += 1;
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
                Stone = true;
                robot.TopDrive.setPower(0);
                robot.BottomDrive.setPower(0);
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                telemetry.update();
                break;
            }
            if(targetVisible == false) { 
                if(LoopNum < 10){
                    sleep(100);
                    telemetry.addData("LoopNum1", LoopNum);
                    telemetry.update();
                    continue;
                }
                if(LoopNum > 11 && LoopNum < 20){
                    sleep(100);
                    telemetry.addData("LoopNum1", LoopNum);
                    telemetry.update();
                    continue;
                }
                if(LoopNum >= 20){
                    telemetry.addData("Visible Target", "none, giving up");
                    telemetry.update();
                    break;
                }
                // At this point in the code, the target is not visible and we have checked exactly 11 times. 
                // We assume the Skystone is not in positions 5 or 6, so we move the 
                //   robot in a position to check positions 3 and 4 and check there 10 times. 
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
            xTarget = -54.5;
            yTarget = 46;
            runtime.reset();
            speed = 0;
            while(!isStopRequested() && opModeIsActive() && 
                (
                   yLocation < yTarget - 0.5 || yLocation > yTarget + 0.5 
                   || xLocation < xTarget - 0.5 || xLocation > xTarget + 0.5)
                ){
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
                leftpower = (yLocation - yTarget)*-0.2+rotationCorrection;
                rightpower = (yLocation - yTarget)*0.2+rotationCorrection;
                if (Alliance==Blue){
                    toppower = (xLocation - xTarget)*-0.2+rotationCorrection;
                    bottompower = (xLocation - xTarget)*0.2+rotationCorrection;
                }            
                if (Alliance==Red){
                    toppower = (xLocation - xTarget)*0.2+rotationCorrection; 
                    bottompower = (xLocation - xTarget)*-0.2+rotationCorrection;
                }
                if (Math.abs(leftpower) > 0.5){
                    rightpower /= Math.abs(leftpower)/0.5/speed;
                    toppower /= Math.abs(leftpower)/0.5/speed;
                    bottompower /= Math.abs(leftpower)/0.5/speed;
                    leftpower /= Math.abs(leftpower)/0.5/speed;
                }
                if (Math.abs(rightpower) > 0.5){
                    leftpower /= Math.abs(rightpower)/0.5/speed;
                    toppower /= Math.abs(rightpower)/0.5/speed;
                    bottompower /= Math.abs(rightpower)/0.5/speed;
                    rightpower /= Math.abs(rightpower)/0.5/speed;
                }
                if (Math.abs(toppower) > 0.5){
                    rightpower /= Math.abs(toppower)/0.5/speed;
                    leftpower /= Math.abs(toppower)/0.5/speed;
                    bottompower /= Math.abs(toppower)/0.5/speed;
                    toppower /= Math.abs(toppower)/0.5/speed;
                }
                if (Math.abs(bottompower) > 0.5){
                    rightpower /= Math.abs(bottompower)/0.5/speed;
                    leftpower /= Math.abs(bottompower)/0.5/speed;
                    toppower /= Math.abs(bottompower)/0.5/speed;
                    bottompower /= Math.abs(bottompower)/0.5/speed;
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
            // At this point, the robot has completed its move so positions 3 and 4 should be visible.
            robot.TopDrive.setPower(0);
            robot.BottomDrive.setPower(0);
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            sleep(300);
            telemetry.update();
            }
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.TopDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        // Normally Skystone has been detected at this point. We have checked positions 3-6. 
        if(WriteLog){
            try{
                FileWriter fw=new FileWriter(FName, true); 
                if(targetVisible == false) {   
                    fw.write("Skystone not detected after checking positions 3-6.\nGiving Up.\nTime: " + opmodeRunTime.seconds() + " seconds\n"); 
                } else {
                    fw.write("Skystone Detected. \nTime: " + opmodeRunTime.seconds() + " seconds\n"); 
                }
                fw.close();
            }catch(Exception e){System.out.println(e);}
        }
        
        LoopNum = 0;
        CenterRunTime.reset();
        stoneXtarget = -8.25;
        stoneYtarget = 7.7;
        stoneHeading = 0;
        //center on stone 1
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
            if (targetVisible) {
                LoopNum += 1;
                telemetry.addData("Loop Number", LoopNum);
                telemetry.update();
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                //start driving to match x and y
                robot.TopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.BottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                TPower = ((translation.get(1) / mmPerInch) - stoneYtarget)*-0.2;
                BPower = ((translation.get(1) / mmPerInch) - stoneYtarget)*0.2;
                LPower = ((translation.get(0) / mmPerInch) - stoneXtarget)*0.25;
                RPower = ((translation.get(0) / mmPerInch) - stoneXtarget)*-0.25;
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
                speed = -0.01*(angles.firstAngle - stoneHeading);
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
                if(WriteLog){
                    try{
                        FileWriter fw=new FileWriter(FName, true);
                        fw.write("Elapsed Time: " + opmodeRunTime.seconds() + " seconds.\n"); 
                        fw.write("x stone:" + (translation.get(0) / mmPerInch) + "\n");  
                        fw.write("y stone:" + (translation.get(1) / mmPerInch) + "\n");
                        fw.write("x location:" + (xLocation) + "\n");
                        fw.write("y location:" + (yLocation) + "\n");
                        fw.write("stone heading:" + (rotation.thirdAngle) + "\n");
                        fw.write("gyro heading:" + (angles.firstAngle) + "\n");
                        fw.write("loop num:" + (LoopNum) + "\n");
                        fw.close();
                    }catch(Exception e){System.out.println(e);}
                }
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
                if(yLocation < 32){
                    encoderDriveForward(DRIVE_SPEED, 7, -7, 0.2);
                }
                if(Alliance == Red && xLocation < -62.5 && translation.get(1) > 0){
                    xLocation = -62;
                    if(WriteLog){
                        try{
                            FileWriter fw=new FileWriter(FName, true);  
                            fw.write("In position to grab Skystone for >0.5sec.\n"); 
                            fw.write("x: " + (translation.get(0) / mmPerInch) + "\n");  
                            fw.write("y: " + (translation.get(1) / mmPerInch) + "\n");  
                            fw.write("Angle: " + rotation.thirdAngle + "\n");
                            fw.write("StonePos: " + StonePos + "\n");
                            fw.close();
                        }catch(Exception e){System.out.println(e);}
                    }
                    break;
                }
                if ((translation.get(1) / mmPerInch) < stoneYtarget + 0.5 && (translation.get(1) / mmPerInch) > stoneYtarget - 0.5 
                            && (translation.get(0) / mmPerInch) > stoneXtarget - 0.5 && (translation.get(0) / mmPerInch) < stoneXtarget + 0.5
                            && angles.firstAngle < stoneHeading + 4 && angles.firstAngle > stoneHeading - 4){
                    if(CenterRunTime.seconds() > 0.5){                    
                        if(WriteLog){
                            try{
                                FileWriter fw=new FileWriter(FName, true);  
                                fw.write("In position to grab Skystone for >0.5sec.\n"); 
                                fw.write("x: " + (translation.get(0) / mmPerInch) + "\n");  
                                fw.write("y: " + (translation.get(1) / mmPerInch) + "\n");  
                                fw.write("Angle: " + rotation.thirdAngle + "\n");
                                fw.write("StonePos: " + StonePos + "\n");
                                fw.close();
                            }catch(Exception e){System.out.println(e);}
                        }
                        break;
                    }
                }else{
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
        xStonePos = xLocation;
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        robot.TopDrive.setPower(0);
        telemetry.addData("StonePos", StonePos);
        
        // Robot is centered on Skystone and we are ready to lower the arm
        //putting arm down
        if(!isStopRequested() && Stone == true){
            ArmPosition = robot.Arm.getCurrentPosition();
            while (!isStopRequested() && robot.Arm.getCurrentPosition()-ArmPosition < 100 ){
                telemetry.addData("arm position",robot.Arm.getCurrentPosition()-ArmPosition);
                telemetry.update();
                robot.Arm.setPower(0.4);
                robot.Arm2.setPower(0.4);
            }
            robot.Arm.setPower(0);
            robot.Arm2.setPower(0);
            
            //robot drives forwards 0.2 inches with the arm down before closing the paddle
            encoderDriveForward(DRIVE_SPEED, -5, 5, 0.2);
            
            for(i = 1; i <= 100; i ++){
                robot.Claw.setPosition(1 + (1.0-1)*i/100);
                sleep(5);
            }
            while (robot.Arm.getCurrentPosition()>ArmPosition + 10){
                telemetry.addData("arm position",robot.Arm.getCurrentPosition()-ArmPosition);
                telemetry.update();
                robot.Arm.setPower(-0.5);
                robot.Arm2.setPower(-0.5);
            }
        }
        robot.FoundPull.setPosition(1.0);
        
        //drive to building side to deliver first skystone

        yTarget = 40;
        xTarget = 15; 
        HeadingTarget = 0;
        runtime.reset();
        speed = 0;
        while (!isStopRequested() && opModeIsActive()){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading=angles.firstAngle;
            speed = runtime.seconds() * robot.maxacceleration;
            if (yLocation > yTarget - 0.5 && yLocation < yTarget + 0.5 && 
                        xLocation > xTarget-0.5 && xLocation < xTarget +0.5 ){
                // Target position has been reached. Stop motors. 
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.TopDrive.setPower(0);
                robot.BottomDrive.setPower(0);
                break;          
            }
            LoopNum++;
            if (Alliance==Blue){
                toppower = (xLocation - xTarget)*-0.3;
                bottompower = (xLocation - xTarget)*0.3;
            }            
            if (Alliance==Red){
                toppower = (xLocation - xTarget)*0.3;
                bottompower = (xLocation - xTarget)*-0.3;
            }
            if(Math.abs(toppower) > 1.0){
                toppower /= Math.abs(toppower);
                bottompower /= Math.abs(bottompower);
            }            
            rotationCorrection = -0.02*(heading-HeadingTarget);
            if (rotationCorrection>0.3){
                rotationCorrection=0.3;
            }
            if (rotationCorrection<-0.3){
                rotationCorrection=-0.3;
            }
            toppower += rotationCorrection;
            bottompower += rotationCorrection;
            leftpower = (yLocation - yTarget)*-0.3+rotationCorrection;
            rightpower = (yLocation - yTarget)*0.3+rotationCorrection;
            if (Math.abs(leftpower) > 1.0){
                rightpower /= Math.abs(leftpower)/speed;
                toppower /= Math.abs(leftpower)/speed;
                bottompower /= Math.abs(leftpower)/speed;
                leftpower /= Math.abs(leftpower)/speed;
            }
            if (Math.abs(rightpower) > 1.0){
                leftpower /= Math.abs(rightpower)/speed;
                toppower /= Math.abs(rightpower)/speed;
                bottompower /= Math.abs(rightpower)/speed;
                rightpower /= Math.abs(rightpower)/speed;
            }
            if (Math.abs(toppower) > 1.0){
                rightpower /= Math.abs(toppower)/speed;
                leftpower /= Math.abs(toppower)/speed;
                bottompower /= Math.abs(toppower)/speed;
                toppower /= Math.abs(toppower)/speed;
            }
            if (Math.abs(bottompower) > 1.0){
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
            telemetry.addData("loopnum",LoopNum);
            telemetry.addData("heading",heading);
            telemetry.addData("x/y", "%.1f / %.1f", xLocation, yLocation);
            telemetry.addData("bearing",bearing);
            telemetry.update();
        }
        
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0); 
        robot.BottomDrive.setPower(0);
        robot.TopDrive.setPower(0);
        if(!isStopRequested() && Stone == true){
            //drop stone
            while (!isStopRequested() && robot.Arm.getCurrentPosition()-ArmPosition < 100 ){
                telemetry.addData("arm position",robot.Arm.getCurrentPosition()-ArmPosition);
                telemetry.update();
                robot.Arm.setPower(0.5);
                robot.Arm2.setPower(0.5);
            }
            robot.Arm.setPower(0);
            robot.Arm2.setPower(0);
            robot.Claw.setPosition(0.2);
            while (!isStopRequested() && robot.Arm.getCurrentPosition()>ArmPosition + 10){
                telemetry.addData("arm position",robot.Arm.getCurrentPosition()-ArmPosition);
                telemetry.update();
                robot.Arm.setPower(-0.5);
                robot.Arm2.setPower(-0.5);
            }
        }
        //drive back to collect second Skystone.
        xTarget = xStonePos + 24;
        yTarget = 40;
        speed = 0;
        runtime.reset();
        while(!isStopRequested() && 
               (
                    yLocation < yTarget - 0.5 || yLocation > yTarget + 0.5 || 
                    xLocation < xTarget - 0.5 || xLocation > xTarget + 0.5
                )){
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
            if (Alliance==Blue){
                toppower = (xLocation - xTarget)*-0.3+rotationCorrection;
                bottompower = (xLocation - xTarget)*0.3+rotationCorrection;
            }            
            if (Alliance==Red){
                toppower = (xLocation - xTarget)*0.3+rotationCorrection; 
                bottompower = (xLocation - xTarget)*-0.3+rotationCorrection;
            }
            if(Math.abs(toppower) > 1.0){
                toppower /= Math.abs(toppower);
                bottompower /= Math.abs(bottompower);
            }
            leftpower = (yLocation - yTarget)*-0.3+rotationCorrection;
            rightpower = (yLocation - yTarget)*0.3+rotationCorrection;
            if (Math.abs(leftpower) > 1.0){
                rightpower /= Math.abs(leftpower)/speed;
                toppower /= Math.abs(leftpower)/speed;
                bottompower /= Math.abs(leftpower)/speed;
                leftpower /= Math.abs(leftpower)/speed;
            }
            if (Math.abs(rightpower) > 1.0){
                leftpower /= Math.abs(rightpower)/speed;
                toppower /= Math.abs(rightpower)/speed;
                bottompower /= Math.abs(rightpower)/speed;
                rightpower /= Math.abs(rightpower)/speed;
            }
            if (Math.abs(toppower) > 1.0){
                rightpower /= Math.abs(toppower)/speed;
                leftpower /= Math.abs(toppower)/speed;
                bottompower /= Math.abs(toppower)/speed;
                toppower /= Math.abs(toppower)/speed;
            }
            if (Math.abs(bottompower) > 1.0){
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
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.TopDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        //look for stone 2
        Stone = false;
        LoopNum = 0;
        while (!isStopRequested() && opModeIsActive()) {
            LoopNum += 1;
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
                    continue;
                }
                if(LoopNum > 11 && LoopNum < 20){
                    telemetry.addData("LoopNum", LoopNum);
                    continue;
                }
                if(LoopNum >= 20){
                    telemetry.addData("Visible Target", "none, giving up");
                    break;
                }
                telemetry.addData("Visible Target", "none");
                robot.TopDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.BottomDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        
                if(WriteLog){
                    try{
                        FileWriter fw=new FileWriter(FName, true);    
                        fw.write("LoopNum = " + LoopNum + "\nSkystone not Detected. \n Driving sideways...\nTime: " + opmodeRunTime.seconds() + " seconds\n"); 
                        fw.close();
                    }catch(Exception e){System.out.println(e);
                }
            }
            //drive to position to see stones 5 and 6
            runtime.reset();
            speed =0;
            while(!isStopRequested() && Math.abs(yLocation) < 44.5 || Math.abs(yLocation) > 45.5 || 
                    (Alliance == Red && Math.abs(xLocation) < 66.5) || (Alliance == Red && Math.abs(xLocation) > 67.5)){
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
                leftpower = (yLocation - 45)*-0.3+rotationCorrection;
                rightpower = (yLocation - 45)*0.3+rotationCorrection;
                if (Alliance==Blue){
                    toppower = (xLocation + 67)*-0.3+rotationCorrection;
                    bottompower = (xLocation + 67)*0.3+rotationCorrection;
                }            
                if (Alliance==Red){
                    toppower = (xLocation + 67)*0.3+rotationCorrection; 
                    bottompower = (xLocation + 67)*-0.3+rotationCorrection;
                }
                if (Math.abs(leftpower) > 1.0){
                    rightpower /= Math.abs(leftpower)/speed;
                    toppower /= Math.abs(leftpower)/speed;
                    bottompower /= Math.abs(leftpower)/speed;
                    leftpower /= Math.abs(leftpower)/speed;
                }
                if (Math.abs(rightpower) > 1.0){
                    leftpower /= Math.abs(rightpower)/speed;
                    toppower /= Math.abs(rightpower)/speed;
                    bottompower /= Math.abs(rightpower)/speed;
                    rightpower /= Math.abs(rightpower)/speed;
                }
                if (Math.abs(toppower) > 1.0){
                    rightpower /= Math.abs(toppower)/speed;
                    leftpower /= Math.abs(toppower)/speed;
                    bottompower /= Math.abs(toppower)/speed;
                    toppower /= Math.abs(toppower)/speed;
                }
                if (Math.abs(bottompower) > 1.0){
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
            sleep(300);
            telemetry.update();
            }
        }
        robot.Claw.setPosition(0.2);
        //center on stone 2
        LoopNum = 0;
        CenterRunTime.reset();
        while (!isStopRequested() && opModeIsActive() && Stone == true) {
            telemetry.addData("loop", "x/y");
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    if(WriteLog){
                        try{
                            FileWriter fw=new FileWriter(FName, true);  
                            fw.write("in position"); 
                            fw.write(String.format("target: %.1f \n", trackable.getName()));
                            fw.close();
                        }catch(Exception e){System.out.println(e);}
                    }
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
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
                telemetry.addData("heading",heading);
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
                rotationCorrection = -0.02*(heading);
                if (rotationCorrection>0.3){
                    rotationCorrection=0.3;
                }
                if (rotationCorrection<-0.3){
                    rotationCorrection=-0.3;
                }
                LPower += rotationCorrection;
                RPower += rotationCorrection;
                BPower += rotationCorrection;
                TPower += rotationCorrection;
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
                if(yLocation < 36){
                    encoderDriveForward(DRIVE_SPEED, 4, -4, 0.5);
                }
                if ((translation.get(1) / mmPerInch) < 8.2 && (translation.get(1) / mmPerInch) > 7.2 
                            && (translation.get(0) / mmPerInch) > -8.75 && (translation.get(0) / mmPerInch) < -7.75
                            && heading < 3 && heading > -3){
                    StonePos = 0;
                    if (Math.abs(xLocation) < 24 && Math.abs(xLocation) >= 16){
                        StonePos = 1;
                    }
                    if (Math.abs(xLocation) < 30 && Math.abs(xLocation) >= 24){
                        StonePos = 2;
                    }
                    if (Math.abs(xLocation) < 36 && Math.abs(xLocation) >= 30){
                        StonePos = 3;
                    }
                    if(WriteLog){
                        try{
                            FileWriter fw=new FileWriter(FName, true);  
                            fw.write("in position");
                            fw.write(String.format("x: %.1f \n", translation.get(0) / mmPerInch));  
                            fw.write(String.format("y: %.1f \n", translation.get(1) / mmPerInch));
                            fw.write(String.format("heading: %.1f \n", rotation.thirdAngle));
                            fw.write(String.format("StonePos: %.1f \n", StonePos));
                            fw.close();
                        }catch(Exception e){System.out.println(e);}
                    }
                    if(CenterRunTime.seconds() > 1){
                        break;
                    }
                }else{
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
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.BottomDrive.setPower(0);
        robot.TopDrive.setPower(0);
        if(Stone){
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
            sleep(5);
        }
        while (!isStopRequested() && robot.Arm.getCurrentPosition()>ArmPosition + 10){
            telemetry.addData("arm position",robot.Arm.getCurrentPosition()-ArmPosition);
            telemetry.update();
            robot.Arm.setPower(-0.5);
            robot.Arm2.setPower(-0.5);
        }
        }
        
        //drive to building side
        yTarget= 40;
        xTarget = 15; 
        runtime.reset();
        sleep = 0;
        while (!isStopRequested() && opModeIsActive()){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading=angles.firstAngle;
            speed = runtime.seconds() * robot.maxacceleration;
            if (Math.abs(yLocation) > yTarget - 0.5 && Math.abs(yLocation) < yTarget + 0.5 && 
                        xLocation > xTarget - 0.5 && xLocation < xTarget + 0.5 && heading < 3 && heading > -3){
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
                toppower = (xLocation - xTarget)*-0.3;
                bottompower = (xLocation - xTarget)*0.3;
            }            
            if (Alliance==Red){
                toppower = (xLocation - xTarget)*0.3;
                bottompower = (xLocation - xTarget)*-0.3;
            }
            if(Math.abs(toppower) > 1.0){
                toppower /= Math.abs(toppower);
                bottompower /= Math.abs(bottompower);
            }
            toppower += rotationCorrection;
            bottompower += rotationCorrection;
            leftpower = (yLocation - yTarget)*-0.3+rotationCorrection;
            rightpower = (yLocation - yTarget)*0.3+rotationCorrection;
            if (Math.abs(leftpower) > 1.0){
                rightpower /= Math.abs(leftpower)/speed;
                toppower /= Math.abs(leftpower)/speed;
                bottompower /= Math.abs(leftpower)/speed;
                leftpower /= Math.abs(leftpower)/speed;
            }
            if (Math.abs(rightpower) > 1.0){
                leftpower /= Math.abs(rightpower)/speed;
                toppower /= Math.abs(rightpower)/speed;
                bottompower /= Math.abs(rightpower)/speed;
                rightpower /= Math.abs(rightpower)/speed;
            }
            if (Math.abs(toppower) > 1.0){
                rightpower /= Math.abs(toppower)/speed;
                leftpower /= Math.abs(toppower)/speed;
                bottompower /= Math.abs(toppower)/speed;
                toppower /= Math.abs(toppower)/speed;
            }
            if (Math.abs(bottompower) > 1.0){
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
        if(Stone == true){
            //drop stone
            while (!isStopRequested() && robot.Arm.getCurrentPosition()-ArmPosition < 100 ){
                telemetry.addData("arm position",robot.Arm.getCurrentPosition()-ArmPosition);
                telemetry.update();
                robot.Arm.setPower(0.5);
                robot.Arm2.setPower(0.5);
            }
            robot.Arm.setPower(0);
            robot.Arm2.setPower(0);
            robot.Claw.setPosition(0.2);
            while (!isStopRequested() && robot.Arm.getCurrentPosition()>ArmPosition + 10){
                telemetry.addData("arm position",robot.Arm.getCurrentPosition()-ArmPosition);
                telemetry.update();
                robot.Arm.setPower(-0.5);
                robot.Arm2.setPower(-0.5);
            }
        }
        xTarget = 0;
        yTarget = 43;
        runtime.reset();
        sleep = 0;
        while(!isStopRequested() && opModeIsActive() && 
            (
                yLocation < yTarget - 0.5 || yLocation > yTarget + 0.5 
                || xLocation < xTarget - 0.5 || xLocation > xTarget + 0.5)
            ){
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
            leftpower = (yLocation - yTarget)*-0.2+rotationCorrection;
            rightpower = (yLocation - yTarget)*0.2+rotationCorrection;
            if (Alliance==Blue){
                toppower = (xLocation - xTarget)*-0.2+rotationCorrection;
                bottompower = (xLocation - xTarget)*0.2+rotationCorrection;
            }            
            if (Alliance==Red){
                toppower = (xLocation - xTarget)*0.2+rotationCorrection; 
                bottompower = (xLocation - xTarget)*-0.2+rotationCorrection;
            }
            if (Math.abs(leftpower) > 1.0){
                rightpower /= Math.abs(leftpower)/sleep;
                toppower /= Math.abs(leftpower)/sleep;
                bottompower /= Math.abs(leftpower)/sleep;
                leftpower /= Math.abs(leftpower)/sleep;
            }
            if (Math.abs(rightpower) > 1.0){
                leftpower /= Math.abs(rightpower)/sleep;
                toppower /= Math.abs(rightpower)/sleep;
                bottompower /= Math.abs(rightpower)/sleep;
                rightpower /= Math.abs(rightpower)/sleep;
            }
            if (Math.abs(toppower) > 1.0){
                rightpower /= Math.abs(toppower)/sleep;
                leftpower /= Math.abs(toppower)/sleep;
                bottompower /= Math.abs(toppower)/sleep;
                toppower /= Math.abs(toppower)/sleep;
            }
            if (Math.abs(bottompower) > 1.0){
                rightpower /= Math.abs(bottompower)/sleep;
                leftpower /= Math.abs(bottompower)/sleep;
                toppower /= Math.abs(bottompower)/sleep;
                bottompower /= Math.abs(bottompower)/sleep;
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
