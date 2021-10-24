package org.firstinspires.ftc.teamcode1920;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.Range;

@TeleOp(name="RevTeleOp")

public class RevTeleOp extends OpMode{

    /* Declare OpMode members. */
    RevRobotHardwareNew robot       = new RevRobotHardwareNew(); // use the class created to define a Pushbot's hardware
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    ARM_SPEED  = 0.4 ; 
    double    ClawSpeed;
    double    FoundSpeed;    
    double                  xLocation = 0; 
    double                  yLocation = 0;
    double                  LocationEncoderT;
    double                  LocationEncoderB;
    double                  LocationEncoderL;
    double                  LocationEncoderR;
    double                  LocationEncoderT1;
    double                  LocationEncoderB1;
    double                  LocationEncoderL1;
    double                  LocationEncoderR1;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        LocationEncoderR = robot.rightDrive.getCurrentPosition();
        LocationEncoderL = robot.leftDrive.getCurrentPosition();
        LocationEncoderT = robot.TopDrive.getCurrentPosition();
        LocationEncoderB = robot.BottomDrive.getCurrentPosition();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double Left;
        double Right;
        double Bottom;
        double Top;
        double TopSpeed;
        double FastestWheel;
        double FoundSpeed;
        double FoundSpeed2;

        TopSpeed = Math.sqrt((gamepad1.left_stick_y * gamepad1.left_stick_y) + (gamepad1.left_stick_x * gamepad1.left_stick_x));
        if (TopSpeed > 1) {
            TopSpeed = 1;
        }
        Bottom = gamepad1.left_stick_x - (gamepad1.right_stick_x * .7);
        Top = -gamepad1.left_stick_x - (gamepad1.right_stick_x * .7);
        Left = gamepad1.left_stick_y - (gamepad1.right_stick_x * .7);
        Right = -gamepad1.left_stick_y - (gamepad1.right_stick_x * .7);
        
        robot.BottomDrive.setPower(Bottom);
        robot.TopDrive.setPower(Top);
        robot.leftDrive.setPower(Left);
        robot.rightDrive.setPower(Right);
        
        telemetry.addData("Left", Left);
        telemetry.addData("Bottom", Bottom);
        telemetry.addData("Top", Top);
        telemetry.addData("Right", Right);
        telemetry.addData("TopSpeed", TopSpeed);
        telemetry.addData("GamePad Lx", gamepad1.left_stick_x);
        telemetry.addData("GamePad Ly", gamepad1.left_stick_y);
        
        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            ClawSpeed -=  .01;
        else if (gamepad1.left_bumper)
            ClawSpeed +=  .01;
        
        if (ClawSpeed > 1){
            ClawSpeed = 1;
        }else if (ClawSpeed < 0) {
            ClawSpeed =0;
        }
        if (gamepad1.right_bumper)
            ClawSpeed -=  .01;
        else if (gamepad1.left_bumper)
            ClawSpeed +=  .01;
        robot.Claw.setPosition(ClawSpeed);
        telemetry.addData("claw",ClawSpeed );
        
        FoundSpeed = 1 - gamepad1.right_trigger;
        FoundSpeed2 = gamepad1.left_trigger;
        
        if (FoundSpeed < robot.FoundPullPos){
            FoundSpeed = robot.FoundPullPos;
        }
        if (FoundSpeed2 > robot.FoundPull2Pos){
            FoundSpeed2 = robot.FoundPull2Pos;
        }
        
        robot.FoundPull.setPosition(FoundSpeed);
        robot.FoundPull2.setPosition(FoundSpeed2);
        telemetry.addData("FoundPull Positions:"," %.2f, %.2f", FoundSpeed, FoundSpeed2);
        
        if (gamepad1.y){
            robot.Capstone.setPosition(0.9);
        }
        if (!gamepad1.y){
            robot.Capstone.setPosition(0.0);
        }

        // Use d-pad to move the arm up (Y) and down (A)
        if (gamepad1.dpad_up){
            robot.Arm.setPower(ARM_SPEED);
            robot.Arm2.setPower(ARM_SPEED);
        }else if (gamepad1.dpad_down){
            robot.Arm.setPower(-ARM_SPEED);
            robot.Arm2.setPower(-ARM_SPEED);
        }else {
            robot.Arm.setPower(0);
            robot.Arm2.setPower(0);
        }
        //Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
