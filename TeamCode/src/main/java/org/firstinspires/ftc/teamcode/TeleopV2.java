package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Driver Control with Pinpoint", group="Test")
public class TeleopV2 extends DriveBase {

    // ===== PINPOINT =====
    GoBildaPinpointDriver odometry;

    @Override
    public void init() {
        super.init(); // keep your original init

        // Initialize Pinpoint
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometry.resetPosAndIMU();

        // Add dashboard telemetry
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

        // ===== UPDATE ODOMETRY =====
        if (odometry != null) {
            odometry.update();
        }

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

        // Shooter powers
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

        // Intake / Outtake buttons
        if (gamepad2.right_bumper) {
            intake();
        } else if (gamepad2.left_bumper) {
            Outtake();
        }
        else {
            // Only use joystick if no bumpers are pressed
            double intakePower = -gamepad2.right_stick_y * RobotConstants.intakeMultiplier;
            robot.Intake.setPower(intakePower);
            robot.Up1.setPower(0);
            robot.Up2.setPower(0);
            robot.Up3.setPower(0);
        }


        if (gamepad2.dpad_down){
            IntakeWithOutTop();
        }

        // ===== TELEMETRY =====
        telemetry.addData("Shooter Power", robot.motor1.getPower());
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        // ===== PINPOINT TELEMETRY =====
        if (odometry != null) {
            telemetry.addData("X (in)", odometry.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", odometry.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", Math.toDegrees(odometry.getHeading(AngleUnit.DEGREES)));
        }

        telemetry.update();
    }
}
