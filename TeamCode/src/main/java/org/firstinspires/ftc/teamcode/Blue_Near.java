package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="BLUE NEAR Villan (DO NOT RUN)", group="Pinpoint")
public class Blue_Near extends LinearOpMode {

    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    DcMotorEx Intake, motor1, motor2;
    CRServo Up1, Up2, Up3;

    GoBildaPinpointDriver odo;
    DriveToPoint nav = new DriveToPoint(this);

    ElapsedTime shooterTimer = new ElapsedTime();
    Pose2D holdPose = null;

    enum StateMachine {
        DRIVE_FORWARD,
        WARM_UP_SHOOTER,
        SHOOT,
        OFF_OF_LINE,
        DONE
    }

    // Forward 60 inches
    static final Pose2D FORWARD_60 =
            new Pose2D(DistanceUnit.INCH, 60, 0, AngleUnit.DEGREES, 0);

    static final Pose2D OFF_OF_LINE =

            new Pose2D(DistanceUnit.INCH, 60, -24, AngleUnit.DEGREES, 90);


    @Override
    public void runOpMode() {

        // Drive motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rb");

        // Shooter / intake
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        motor1 = hardwareMap.get(DcMotorEx.class, "SL");
        motor2 = hardwareMap.get(DcMotorEx.class, "SR");

        Up1 = hardwareMap.get(CRServo.class, "Up1");
        Up2 = hardwareMap.get(CRServo.class, "Up2");
        Up3 = hardwareMap.get(CRServo.class, "Up3");

        // Motor setup
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Pinpoint
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-3.5, -5.75, DistanceUnit.INCH);
        odo.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        odo.resetPosAndIMU();
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine state = StateMachine.DRIVE_FORWARD;

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            odo.update();

            switch (state) {

                case DRIVE_FORWARD:
                    if (nav.driveTo(odo.getPosition(), FORWARD_60, 0.7, 0)) {
                        holdPose = odo.getPosition();




                        shooterTimer.reset();
                        state = StateMachine.WARM_UP_SHOOTER;
                    }
                    break;

                case WARM_UP_SHOOTER:
                    nav.driveTo(odo.getPosition(), holdPose, 0.4, 0);

                    motor1.setPower(-0.3);
                    motor2.setPower(0.3);

                    if (shooterTimer.seconds() >= 5.0) {
                        shooterTimer.reset();
                        state = StateMachine.SHOOT;
                    }
                    break;

                case SHOOT:
                    nav.driveTo(odo.getPosition(), holdPose, 0.4, 0);

                    motor1.setPower(-0.3);
                    motor2.setPower(0.3);
                    Intake.setPower(-0.75);
                    Up1.setPower(-1);
                    Up2.setPower(-1);
                    Up3.setPower(-1);

                    if (shooterTimer.seconds() >= 7.0) {
                        motor1.setPower(0);
                        motor2.setPower(0);
                        Intake.setPower(0);
                        Up1.setPower(0);
                        Up2.setPower(0);
                        Up3.setPower(0);

                        state = StateMachine.OFF_OF_LINE;
                    }
                    break;

                case OFF_OF_LINE:
                    if (nav.driveTo(odo.getPosition(), OFF_OF_LINE, 0.6, 0)) {
                        holdPose = odo.getPosition();
                        state = StateMachine.DONE;
                    }
                    break;

                case DONE:
                    nav.driveTo(odo.getPosition(), holdPose, 0.4, 0);
                    break;
            }

            // Apply drive powers
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            Pose2D pos = odo.getPosition();
            telemetry.addData("State", state);
            telemetry.addData("X", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y", pos.getY(DistanceUnit.INCH));
            telemetry.addData("H", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
