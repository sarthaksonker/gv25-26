package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveBase extends OpMode {

    public devices robot = new devices();
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    // Stop all drivetrain motors
    public void drivestop() {
        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }

    public void intake() {
        robot.Up1.setPower(-0.8);
        robot.Up2.setPower(1);
        robot.V1.setPower(1);
        robot.V2.setPower(-1);
        robot.Intake.setPower(-1);
    }
    public void intake_stop() {
        robot.Up1.setPower(0);
        robot.Up2.setPower(0);
        robot.V1.setPower(0);
        robot.V2.setPower(0);
        robot.Intake.setPower(0);
    }

    public void Outtake(){
        robot.Up1.setPower(1);
        robot.Up2.setPower(-1);
        robot.V1.setPower(-1);
        robot.V2.setPower(1);
        robot.Intake.setPower(1);
    }

    public void Outtake_stop(){
        robot.Up1.setPower(0);
        robot.Up2.setPower(0);
        robot.V1.setPower(0);
        robot.V2.setPower(0);
        robot.Intake.setPower(0);
    }



    public void omniDrive(double y, double x, double rx, double factor) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        robot.leftFrontDrive.setPower(frontLeftPower * factor);
        robot.leftBackDrive.setPower(backLeftPower * factor);
        robot.rightFrontDrive.setPower(frontRightPower * factor);
        robot.rightBackDrive.setPower(backRightPower * factor);
    }

    @Override
    public void loop() {
    }
}
