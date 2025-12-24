package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Drive Test", group="test")
public class Drive_Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize MecanumDrive (just using hardwareMap)
        MecanumDrive drive = new MecanumDrive(hardwareMap, null); // we won't use pose

        telemetry.addLine("Press play to start motor test");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("Running motor test...");
        telemetry.update();

        // Forward test
        drive.leftFrontDrive.setPower(0.3);
        drive.leftBackDrive.setPower(0.3);
        drive.rightFrontDrive.setPower(0.3);
        drive.rightBackDrive.setPower(0.3);
        sleep(2000);

        // Backward test
        drive.leftFrontDrive.setPower(-0.3);
        drive.leftBackDrive.setPower(-0.3);
        drive.rightFrontDrive.setPower(-0.3);
        drive.rightBackDrive.setPower(-0.3);
        sleep(2000);

        // Strafe right
        drive.leftFrontDrive.setPower(0.3);
        drive.leftBackDrive.setPower(-0.3);
        drive.rightFrontDrive.setPower(-0.3);
        drive.rightBackDrive.setPower(0.3);
        sleep(2000);

        // Strafe left
        drive.leftFrontDrive.setPower(-0.3);
        drive.leftBackDrive.setPower(0.3);
        drive.rightFrontDrive.setPower(0.3);
        drive.rightBackDrive.setPower(-0.3);
        sleep(2000);

        // Rotate clockwise
        drive.leftFrontDrive.setPower(0.3);
        drive.leftBackDrive.setPower(0.3);
        drive.rightFrontDrive.setPower(-0.3);
        drive.rightBackDrive.setPower(-0.3);
        sleep(2000);

        // Stop all motors
        drive.leftFrontDrive.setPower(0);
        drive.leftBackDrive.setPower(0);
        drive.rightFrontDrive.setPower(0);
        drive.rightBackDrive.setPower(0);


        telemetry.update();
    }
}
