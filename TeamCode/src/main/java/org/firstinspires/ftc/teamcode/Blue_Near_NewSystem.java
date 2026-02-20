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

@Autonomous(name="BLUE NEAR Villain NewSystem", group="Pinpoint")
public class Blue_Near_NewSystem extends LinearOpMode {

    DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    DcMotorEx Intake, shooterLeft, shooterRight;
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

    // --- Poses ---
    static final Pose2D FORWARD_60 =
            new Pose2D(DistanceUnit.INCH, 60, 0, AngleUnit.DEGREES, 0);

    static final Pose2D Ready1 =
            new Pose2D(DistanceUnit.INCH, 78, 0, AngleUnit.DEGREES, -135);

    static final Pose2D GET_STACK =
            new Pose2D(DistanceUnit.INCH, 53, -42, AngleUnit.DEGREES, -135);

    static final Pose2D MOVE_TO_SHOOT_2 =
            new Pose2D(DistanceUnit.INCH, 60, 0, AngleUnit.DEGREES, 0);

    static final Pose2D OFF_OF_LINE =
            new Pose2D(DistanceUnit.INCH, 40, -28, AngleUnit.DEGREES, -135);

    // --- Shooter powers ---
    int power1 = 690;   // normal shot
    int power2 = 980;   // optional high shot
    int power3 = 1100;  // optional max shot
    int power4 = 760;   // optional lower shot

    @Override
    public void runOpMode() {

        // --- Hardware mapping ---
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rb");

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "SL");
        shooterRight = hardwareMap.get(DcMotorEx.class, "SR");

        Up1 = hardwareMap.get(CRServo.class, "Up1");
        Up2 = hardwareMap.get(CRServo.class, "Up2");
        Up3 = hardwareMap.get(CRServo.class, "Up3");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                    shooterLeft.setVelocity(-power1);
                    shooterRight.setVelocity(power1);
                    if (shooterTimer.seconds() >= 6.0) {
                        shooterTimer.reset();
                        state = StateMachine.SHOOT;
                    }
                    break;

                case SHOOT:
                    nav.driveTo(odo.getPosition(), holdPose, 0.4, 0);

                    // Only shoot when velocity is in range 650–710
                    if (shooterLeft.getVelocity() >= 650 && shooterLeft.getVelocity() <= 710 &&
                            shooterRight.getVelocity() >= 650 && shooterRight.getVelocity() <= 710) {
                        shoot(); // fires rings
                        shooterTimer.reset();
                        state = StateMachine.Ready1;
                    }
                    break;

                case Ready1:
                    if (nav.driveTo(odo.getPosition(), Ready1, 0.4, 0)
                            || shooterTimer.seconds() > 2.0) {
                        holdPose = odo.getPosition();
                        shooterTimer.reset();
                        state = StateMachine.GET_STACK;
                    }
                    break;

                case GET_STACK:
                    nav.driveTo(odo.getPosition(), GET_STACK, 0.2, 0);

                    if (shooterTimer.seconds() <= 3) {
                        Intake.setPower(-1);
                        Up1.setPower(-1);
                        Up2.setPower(-1);
                        Up3.setPower(-1);
                    } else if (shooterTimer.seconds() <= 6.0) {
                        Intake.setPower(0);
                        Up1.setPower(0);
                        Up2.setPower(0);
                        Up3.setPower(0);
                    } else {
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
                    shooterLeft.setVelocity(-power1);
                    shooterRight.setVelocity(power1);

                    // Only shoot when velocity is in range 650–710
                    if (shooterLeft.getVelocity() >= 650 && shooterLeft.getVelocity() <= 710 &&
                            shooterRight.getVelocity() >= 650 && shooterRight.getVelocity() <= 710 &&
                            shooterTimer.seconds() >= 3.0) {
                        shoot();
                        shooterLeft.setVelocity(0);
                        shooterRight.setVelocity(0);
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

            // Apply drive powers
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
            telemetry.addData("Shooter Vel L", shooterLeft.getVelocity());
            telemetry.addData("Shooter Vel R", shooterRight.getVelocity());
            telemetry.update();
        }
    }

    public void shoot() {
        Up1.setPower(-1);
        Up2.setPower(-1);
        Up3.setPower(-1);
        Intake.setPower(-1);
    }
}
