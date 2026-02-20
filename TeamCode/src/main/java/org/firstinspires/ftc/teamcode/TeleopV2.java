package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp(name="Driver Control Blue", group="Test")
public class TeleopV2 extends DriveBase {

    @Override
    public void init() {
        super.init(); // initialize motors in DriveBase
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // --- Set RGB light to blue ---
        robot.rgbLight.setPosition(0.611);
    }

    public double square(double value){
        return value * Math.abs(value);
    }

    @Override
    public void loop() {

        double leftStickX = square(-gamepad1.left_stick_x);
        double leftStickY = square(gamepad1.left_stick_y);
        double rightStickX = square(-gamepad1.right_stick_x);

        // --- Drive ---
        if (gamepad1.left_bumper) {
            omniDrive(leftStickY, leftStickX, rightStickX, RobotConstants.slowMultiplier);
        } else if (gamepad1.right_bumper) {
            omniDrive(leftStickY, leftStickX, rightStickX, RobotConstants.turboMultiplier);
        } else {
            omniDrive(leftStickY, leftStickX, rightStickX, RobotConstants.normalMultiplier);
        }

        // --- Shooter powers ---
        if(gamepad2.a){
            robot.motor1.setVelocity(RobotConstants.power1);
            robot.motor2.setVelocity(RobotConstants.power1);
        }
        else if (gamepad2.x) {
            robot.motor1.setVelocity(RobotConstants.power4);
            robot.motor2.setVelocity(RobotConstants.power4);
        }
        else if (gamepad2.y) {
            robot.motor1.setVelocity(RobotConstants.power3);
            robot.motor2.setVelocity(RobotConstants.power3);
        }
        else if (gamepad2.b) {
            robot.motor1.setVelocity(RobotConstants.power2);
            robot.motor2.setVelocity(RobotConstants.power2);
        }
        else {
            robot.motor1.setVelocity(0);
            robot.motor2.setVelocity(0);
        }

        // --- Intake / Outtake ---
        if (gamepad2.right_bumper) {
            intake();
        } else if (gamepad2.left_bumper) {
            Outtake();
        } else {
            robot.Intake.setPower(0);
            robot.Up1.setPower(0);
            robot.Up2.setPower(0);
            robot.Up3.setPower(0);
        }

        // --- Right trigger shooting logic ---
        if (gamepad2.right_trigger > 0.75) {
            robot.Intake.setPower(-1);

            if (gamepad2.a &&
                    robot.motor1.getVelocity() >= 650 && robot.motor1.getVelocity() <= 710 &&
                    robot.motor2.getVelocity() >= 650 && robot.motor2.getVelocity() <= 710) {
                shoot(); // power1 shot
            }

            if (gamepad2.x &&
                    robot.motor1.getVelocity() >= 740 && robot.motor1.getVelocity() <= 780 &&
                    robot.motor2.getVelocity() >= 740 && robot.motor2.getVelocity() <= 780) {
                shoot(); // power4 shot
            }

            if (gamepad2.y &&
                    robot.motor1.getVelocity() >= 1080 && robot.motor1.getVelocity() <= 1120 &&
                    robot.motor2.getVelocity() >= 1080 && robot.motor2.getVelocity() <= 1120) {
                shoot(); // power3 shot
            }

            if (gamepad2.b &&
                    robot.motor1.getVelocity() >= 960 && robot.motor1.getVelocity() <= 1000 &&
                    robot.motor2.getVelocity() >= 960 && robot.motor2.getVelocity() <= 1000) {
                shoot(); // power2 shot
            } else {
                robot.Intake.setPower(0);
            }
        }

        if (gamepad2.dpad_down){
            IntakeWithOutTop();
        }

        // --- Telemetry ---
        telemetry.addData("Shooter Power", robot.motor1.getPower());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    // --- Shoot method ---
    public void shoot() {
        robot.Up1.setPower(-1);
        robot.Up2.setPower(-1);
        robot.Up3.setPower(-1);
        robot.Intake.setPower(-1);
    }
}
