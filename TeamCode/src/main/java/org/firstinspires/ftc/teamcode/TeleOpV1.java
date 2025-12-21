package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Driver Control  ...", group="Test")
public class TeleOpV1 extends DriveBase {

    // Shooter Speeds
    public double power1 = 0.30;
    public double power2 = 0.35;
    public double power3 = 0.45;
    public double power4 = 0.50;


    public double square(double value){
        return value * Math.abs(value);
    }

    @Override
    public void loop() {


        double leftStickX = square(-gamepad1.left_stick_x);
        double leftStickY = square(gamepad1.left_stick_y);
        double rightStickX = square(-gamepad1.right_stick_x);

        if (gamepad1.left_bumper) {
            omniDrive(leftStickY, leftStickX, rightStickX, 0.35);
        } else if (gamepad1.right_bumper) {
            omniDrive(leftStickY, leftStickX, rightStickX, 1);
        } else {
            omniDrive(leftStickY, leftStickX, rightStickX, 0.75);
        }



        if(gamepad2.a){
            robot.motor1.setPower(power1);
            robot.motor2.setPower(power1);
        }
        else if (gamepad2.x) {
            robot.motor1.setPower(power2);
            robot.motor2.setPower(power2);
        }
        else if (gamepad2.y) {
            robot.motor1.setPower(power3);
            robot.motor2.setPower(power3);
        }
        else if (gamepad2.b) {
            robot.motor1.setPower(power4);
            robot.motor2.setPower(power4);
        }
        else {
            robot.motor1.setPower(0);
            robot.motor2.setPower(0);
        }


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


        robot.Worm.setPower(gamepad2.left_stick_y);



        robot.Intake.setPower(-gamepad2.right_stick_y);



        telemetry.addData("Shooter Power", robot.motor1.getPower());
        telemetry.addData("Intake", robot.Intake.getPower());
        telemetry.addData("Worm", robot.Worm.getPower());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
