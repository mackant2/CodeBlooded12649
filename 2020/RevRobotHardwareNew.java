
package org.firstinspires.ftc.teamcode1920;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RevRobotHardwareNew
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  BottomDrive  = null;
    public DcMotor  TopDrive  = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  Arm  = null;
    public DcMotor  Arm2  = null;
    public DcMotor  Light = null;
    public Servo    Claw = null;
    public Servo    FoundPull = null;
    public Servo    FoundPull2 = null;
    public Servo    Capstone = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double FoundPullPos    = 0.53; // Minimum position when puller is all the way down
    public static final double FoundPull2Pos   = 0.59; // Maximum position when puller is all the way down
    public static final double maxacceleration   = 2; // Maximum acceleration the robot can go 

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RevRobotHardwareNew(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "LeftDrive");
        BottomDrive = hwMap.get(DcMotor.class, "BottomDrive");
        TopDrive  = hwMap.get(DcMotor.class, "TopDrive");
        rightDrive = hwMap.get(DcMotor.class, "RightDrive");
        Arm = hwMap.get(DcMotor.class, "Arm");
        Arm2 = hwMap.get(DcMotor.class, "Arm2");
        Light = hwMap.get(DcMotor.class, "Light");
        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        BottomDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        TopDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        Arm.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        Arm2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        leftDrive.setPower(0);
        BottomDrive.setPower(0);
        TopDrive.setPower(0);
        rightDrive.setPower(0);
        Arm.setPower(0);
        Arm2.setPower(0);
        Light.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BottomDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TopDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        Claw = hwMap.get(Servo.class, "Claw");
        Capstone = hwMap.get(Servo.class, "Capstone");
        FoundPull = hwMap.get(Servo.class, "FoundPull");
        FoundPull2 = hwMap.get(Servo.class, "FoundPull2");
    }
 }

