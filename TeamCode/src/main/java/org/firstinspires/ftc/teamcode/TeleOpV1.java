package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp(name="Driver Control", group="Test")
public class TeleOpV1 extends DriveBase {

    @Override
    public void init() {
        super.init(); // initialize motors in DriveBase
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
    }

    public double square(double value){
        return value * Math.abs(value);
    }

    @Override
    public void loop() {

        double leftStickX = square(-gamepad1.left_stick_x);
        double leftStickY = square(gamepad1.left_stick_y);
        double rightStickX = square(-gamepad1.right_stick_x);

        // Drive with constants
        if (gamepad1.left_bumper) {
            omniDrive(leftStickY, leftStickX, rightStickX, RobotConstants.slowMultiplier);
        } else if (gamepad1.right_bumper) {
            omniDrive(leftStickY, leftStickX, rightStickX, RobotConstants.turboMultiplier);
        } else {
            omniDrive(leftStickY, leftStickX, rightStickX, RobotConstants.normalMultiplier);
        }

        // Shooter powers with constants
        if(gamepad2.a){
            robot.motor1.setPower(RobotConstants.power1);
            robot.motor2.setPower(RobotConstants.power1);
        }
        else if (gamepad2.x) {
            robot.motor1.setPower(RobotConstants.power2);
            robot.motor2.setPower(RobotConstants.power2);
        }
        else if (gamepad2.y) {
            robot.motor1.setPower(RobotConstants.power3);
            robot.motor2.setPower(RobotConstants.power3);
        }
        else if (gamepad2.b) {
            robot.motor1.setPower(RobotConstants.power4);
            robot.motor2.setPower(RobotConstants.power4);
        }
        else {
            robot.motor1.setPower(0);
            robot.motor2.setPower(0);
        }

        // Intake / Outtake
        if (gamepad2.right_bumper){
            intake();
        }
        else {
            intake_stop();
        }

        if (gamepad2.left_bumper) {
            Outtake();
        }
        else {
            Outtake_stop();
        }

        // Worm motor with multiplier
        robot.Worm.setPower(gamepad2.left_stick_y * RobotConstants.wormMultiplier);

        // Intake motor with multiplier
        double intakePower = -gamepad2.right_stick_y * RobotConstants.intakeMultiplier;
        robot.Intake.setPower(intakePower);

        // TELEMETRY
        telemetry.addData("Shooter Power", robot.motor1.getPower());
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Worm Power", robot.Worm.getPower());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
