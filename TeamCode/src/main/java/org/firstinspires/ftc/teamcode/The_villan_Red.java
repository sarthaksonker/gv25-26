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

import java.util.Locale;

@Autonomous(name="THE VILLIAN Red PP", group="Pinpoint")
public class The_villan_Red extends LinearOpMode {

    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    DcMotorEx Intake, motor1, motor2;
    CRServo Up1, Up2, Up3;

    GoBildaPinpointDriver odo;
    DriveToPoint nav = new DriveToPoint(this);

    ElapsedTime shooterTimer = new ElapsedTime();
    Pose2D holdPose = null;

    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_TO_TARGET_1,
        WARM_UP_SHOOTER,
        SHOOT,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        HOLD_AT_TARGET_4,
        DRIVE_TO_TARGET_5,
        SHOOT2,
        DRIVE_TO_TARGET_2,
        DONE
    }

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.INCH, 110, 0, AngleUnit.DEGREES, 135);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.INCH, 50, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.INCH, 41, 0, AngleUnit.DEGREES, -90);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.INCH, 41, -48, AngleUnit.DEGREES, -90);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.INCH, 110, 0, AngleUnit.DEGREES, 135);

    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rb");

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        motor1 = hardwareMap.get(DcMotorEx.class, "SL");
        motor2 = hardwareMap.get(DcMotorEx.class, "SR");

        Up1 = hardwareMap.get(CRServo.class, "Up1");
        Up2 = hardwareMap.get(CRServo.class, "Up2");
        Up3 = hardwareMap.get(CRServo.class, "Up3");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-3.5, -5.75, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            odo.update();

            switch (stateMachine) {

                case WAITING_FOR_START:
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;

                case DRIVE_TO_TARGET_1:
                    // Spin shooters while driving
                    motor1.setPower(-0.3);
                    motor2.setPower(0.3);

                    if (nav.driveTo(odo.getPosition(), TARGET_1, 1.0, 0)) {
                        holdPose = odo.getPosition();
                        shooterTimer.reset();
                        stateMachine = StateMachine.WARM_UP_SHOOTER;
                    }
                    break;

                case WARM_UP_SHOOTER:
                    nav.driveTo(odo.getPosition(), holdPose, 1.0, 0);
                    motor1.setPower(-0.3);
                    motor2.setPower(0.3);

                    if (shooterTimer.seconds() >= 2.0) { //
                        shooterTimer.reset();
                        stateMachine = StateMachine.SHOOT;
                    }
                    break;

                case SHOOT:
                    nav.driveTo(odo.getPosition(), holdPose, 1.0, 0);
                    motor1.setPower(-0.3);
                    motor2.setPower(0.3);
                    Intake.setPower(-0.75);
                    Up1.setPower(-1);
                    Up2.setPower(-1);
                    Up3.setPower(-1);

                    if (shooterTimer.seconds() >= 5.0) {
                        motor1.setPower(0);
                        motor2.setPower(0);
                        Intake.setPower(0);
                        Up1.setPower(0);
                        Up2.setPower(0);
                        Up3.setPower(0);
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;

                case DRIVE_TO_TARGET_3:
                    if (nav.driveTo(odo.getPosition(), TARGET_3, 1.0, 0)) {
                        holdPose = odo.getPosition();
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;

                case DRIVE_TO_TARGET_4:
                    if (nav.driveTo(odo.getPosition(), TARGET_4, 1.0, 0)) {
                        Intake.setPower(-1);
                        Up1.setPower(-1);
                        holdPose = odo.getPosition();
                        shooterTimer.reset();
                        stateMachine = StateMachine.HOLD_AT_TARGET_4;
                    }
                    break;

                case HOLD_AT_TARGET_4:
                    nav.driveTo(odo.getPosition(), holdPose, 1.0, 0);
                    if (shooterTimer.seconds() >= 1.5) {
                        Intake.setPower(0);
                        Up1.setPower(0);
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;

                case DRIVE_TO_TARGET_5:
                    motor1.setPower(-0.3);
                    motor2.setPower(0.3);
                    Intake.setPower(-0.75);
                    Up1.setPower(-1);

                    if (nav.driveTo(odo.getPosition(), TARGET_5, 1.0, 0)) {
                        holdPose = odo.getPosition();
                        shooterTimer.reset();
                        Intake.setPower(0);
                        Up1.setPower(0);
                        stateMachine = StateMachine.SHOOT2;
                    }
                    break;

                case SHOOT2:
                    nav.driveTo(odo.getPosition(), holdPose, 1.0, 0);
                    motor1.setPower(-0.3);
                    motor2.setPower(0.3);
                    Intake.setPower(-0.75);
                    Up1.setPower(-1);
                    Up2.setPower(-1);
                    Up3.setPower(-1);

                    if (shooterTimer.seconds() >= 4.5) {
                        motor1.setPower(0);
                        motor2.setPower(0);
                        Intake.setPower(0);
                        Up1.setPower(0);
                        Up2.setPower(0);
                        Up3.setPower(0);
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    }
                    break;

                case DRIVE_TO_TARGET_2:
                    if (nav.driveTo(odo.getPosition(), TARGET_2, 1.0, 0)) {
                        holdPose = odo.getPosition();
                        stateMachine = StateMachine.DONE;
                    }
                    break;

                case DONE:
                    nav.driveTo(odo.getPosition(), holdPose, 1.0, 0);
                    break;
            }

            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            Pose2D pos = odo.getPosition();
            telemetry.addData("State", stateMachine);
            telemetry.addData(
                    "Pose",
                    String.format(Locale.US, "X: %.1f  Y: %.1f  H: %.1f",
                            pos.getX(DistanceUnit.INCH),
                            pos.getY(DistanceUnit.INCH),
                            pos.getHeading(AngleUnit.DEGREES))
            );
            telemetry.update();
        }
    }
}

