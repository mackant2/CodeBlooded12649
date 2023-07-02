package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware
{
    public DcMotor  RightFront;
    public DcMotor  LeftFront;
    public DcMotor  RightBack;
    public DcMotor  LeftBack;

    HardwareMap hardwareMap;
    
    public void init(HardwareMap hardwareMap) {

        RightFront  = hardwareMap.get(DcMotor.class, "RightFront");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        RightBack   = hardwareMap.get(DcMotor.class, "RightBack");
        LeftBack   = hardwareMap.get(DcMotor.class, "LeftBack");

        RightFront.setDirection(DcMotor.Direction.REVERSE); 
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);

        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightBack.setPower(0);
        LeftBack.setPower(0);

        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
    }
 }
