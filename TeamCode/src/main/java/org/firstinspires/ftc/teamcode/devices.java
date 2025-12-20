package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class devices {

    public DcMotorEx leftFrontDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightFrontDrive;
    public DcMotorEx rightBackDrive;

    public DcMotorEx Worm;
    public DcMotorEx Intake;

    public DcMotorEx motor1;
    public DcMotorEx motor2;

    public CRServo Up1;
    public CRServo Up2;

    public CRServo V1;
    public CRServo V2;

    public void init(HardwareMap hwMap) {


        leftFrontDrive  = hwMap.get(DcMotorEx.class, "lf");
        leftBackDrive   = hwMap.get(DcMotorEx.class, "lb");
        rightFrontDrive = hwMap.get(DcMotorEx.class, "rf");
        rightBackDrive  = hwMap.get(DcMotorEx.class, "rb");


        Worm   = hwMap.get(DcMotorEx.class, "Worm");
        Intake = hwMap.get(DcMotorEx.class, "Intake");

        motor1 = hwMap.get(DcMotorEx.class, "SL");
        motor2 = hwMap.get(DcMotorEx.class, "SR");

        Up1 = hwMap.get(CRServo.class, "Up1");
        Up2 = hwMap.get(CRServo.class, "Up2");

        V1 = hwMap.get(CRServo.class, "V1");
        V2 = hwMap.get(CRServo.class, "V2");


        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Zero power behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Worm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
