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

@Autonomous(name="RED NEAR Villain", group="Pinpoint")
public class Red_Near_Villian extends LinearOpMode {

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
        Ready1,
        GET_STACK,
        MOVE_TO_SHOOT_2,
        SHOOT2,
        OFF_OF_LINE,
        DONE
    }

    static final Pose2D FORWARD_60 =
            new Pose2D(DistanceUnit.INCH, 60, 0, AngleUnit.DEGREES, 0);

    static final Pose2D Ready1 =
            new Pose2D(DistanceUnit.INCH, 78, 0, AngleUnit.DEGREES, 135);

    static final Pose2D GET_STACK =
            new Pose2D(DistanceUnit.INCH, 40, 28, AngleUnit.DEGREES, 135);

    static final Pose2D MOVE_TO_SHOOT_2 =
            new Pose2D(DistanceUnit.INCH, 60, 0, AngleUnit.DEGREES, 0);

    static final Pose2D OFF_OF_LINE =
            new Pose2D(DistanceUnit.INCH, 40, 28, AngleUnit.DEGREES, 135);

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

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                    if (nav.driveTo(odo.getPosition(), FORWARD_60, 0.8, 0)) {
                        holdPose = odo.getPosition();
                        shooterTimer.reset();
                        state = StateMachine.WARM_UP_SHOOTER;
                    }
                    break;

                case WARM_UP_SHOOTER:
                    nav.driveTo(odo.getPosition(), holdPose, 0.4, 0);
                    motor1.setPower(-0.27);
                    motor2.setPower(0.27);
                    if (shooterTimer.seconds() >= 5.0) {
                        shooterTimer.reset();
                        state = StateMachine.SHOOT;
                    }
                    break;

                case SHOOT:
                    nav.driveTo(odo.getPosition(), holdPose, 0.4, 0);
                    motor1.setPower(-0.29);
                    motor2.setPower(0.29);
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
                        state = StateMachine.Ready1;
                    }
                    break;

                case Ready1:
                    if (nav.driveTo(odo.getPosition(), Ready1, 0.4, 0)) {
                        holdPose = odo.getPosition();
                        shooterTimer.reset();
                        state = StateMachine.GET_STACK;
                    }
                    break;

                case GET_STACK:
                    nav.driveTo(odo.getPosition(), GET_STACK, 0.6, 0);

                    // Intake for first 2 seconds
                    if (shooterTimer.seconds() <= 1.3) {
                        Intake.setPower(-0.75);
                        Up1.setPower(-1);
                        Up2.setPower(-1);
                        Up3.setPower(-1);
                    }
                    // Stop all motors for next 3 seconds
                    else if (shooterTimer.seconds() <= 5.0) {
                        Intake.setPower(0);
                        Up1.setPower(0);
                        Up2.setPower(0);
                        Up3.setPower(0);
                    }
                    else {
                        holdPose = odo.getPosition();
                        shooterTimer.reset();
                        state = StateMachine.MOVE_TO_SHOOT_2;
                    }
                    break;

                case MOVE_TO_SHOOT_2:
                    if (nav.driveTo(odo.getPosition(), MOVE_TO_SHOOT_2, 0.8, 0)) {
                        holdPose = odo.getPosition();
                        shooterTimer.reset();
                        state = StateMachine.SHOOT2;
                    }
                    break;

                case SHOOT2:
                    nav.driveTo(odo.getPosition(), holdPose, 0.4, 0);
                    motor1.setPower(-0.27);
                    motor2.setPower(0.27);
                    Intake.setPower(-0.75);
                    Up1.setPower(-1);
                    Up2.setPower(-1);
                    Up3.setPower(-1);
                    if (shooterTimer.seconds() >= 3.0) { // hold 3 sec
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
                    if (nav.driveTo(odo.getPosition(), OFF_OF_LINE, 0.8, 0)) {
                        holdPose = odo.getPosition();
                        state = StateMachine.DONE;
                    }
                    break;

                case DONE:
                    nav.driveTo(odo.getPosition(), holdPose, 0.4, 0);
                    break;
            }

            // Apply drive powers every loop so robot moves
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            // Telemetry
            Pose2D pos = odo.getPosition();
            telemetry.addData("State", state);
            telemetry.addData("X", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y", pos.getY(DistanceUnit.INCH));
            telemetry.addData("H", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
