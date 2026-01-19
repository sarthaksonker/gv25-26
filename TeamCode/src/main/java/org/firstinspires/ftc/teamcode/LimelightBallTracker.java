package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name="Limelight Aim Assist", group="Limelight")
public class LimelightBallTracker extends LinearOpMode {

    // Drive motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Limelight
    private Limelight3A limelight;

    // Settings
    private static final double ROTATION_GAIN = 0.03;   // how fast it turns to target
    private static final double MAX_ROTATE_POWER = 0.6;
    private static final double MIN_ROTATE_POWER = 0.12;

    private boolean aimAssistEnabled = true;

    @Override
    public void runOpMode() {

        // MOTOR SETUP
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "lb");
        leftBackDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // LIMELIGHT SETUP
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("AIM ASSIST ON (press A to disable, B to re-enable)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // TOGGLE
            if (gamepad1.a) {
                aimAssistEnabled = false;
                sleep(250);
            }
            if (gamepad1.b) {
                aimAssistEnabled = true;
                sleep(250);
            }

            // DRIVER INPUTS
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double manualRotate = gamepad1.right_stick_x;

            double autoRotate = 0;

            // === AIM ASSIST LOGIC ===
            if (aimAssistEnabled) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

                    if (detections != null && !detections.isEmpty()) {

                        // Pick nearest (largest area)
                        LLResultTypes.DetectorResult best = null;
                        double bestArea = 0;

                        for (LLResultTypes.DetectorResult d : detections) {
                            double area = d.getTargetArea();
                            if (area > bestArea) {
                                bestArea = area;
                                best = d;
                            }
                        }

                        if (best != null) {
                            double tx = best.getTargetXDegrees();

                            // rotation only — NO confidence threshold
                            autoRotate = Range.clip(tx * ROTATION_GAIN, -MAX_ROTATE_POWER, MAX_ROTATE_POWER);

                            // enforce minimum turn power
                            if (Math.abs(autoRotate) < MIN_ROTATE_POWER && Math.abs(autoRotate) > 0) {
                                autoRotate = Math.signum(autoRotate) * MIN_ROTATE_POWER;
                            }

                            telemetry.addData("Tracking", "YES");
                            telemetry.addData("tx", tx);
                        }
                    }
                }
            }

            // FINAL ROTATION = DRIVER + AIM ASSIST
            double finalRotate = manualRotate + autoRotate;

            // MECANUM DRIVE
            mecanumDrive(forward, strafe, finalRotate);

            // TELEMETRY
            telemetry.addData("Aim Assist", aimAssistEnabled ? "ON" : "OFF");
            telemetry.addData("Manual Rotate", manualRotate);
            telemetry.addData("Auto Rotate", autoRotate);
            telemetry.addData("Final Rotate", finalRotate);
            telemetry.update();
        }

        limelight.stop();
    }

    // Mecanum drive (unchanged from your original)
    private void mecanumDrive(double forward, double strafe, double rotate) {

        double lf = forward + strafe + rotate;
        double rf = forward - strafe - rotate;
        double lb = forward - strafe + rotate;
        double rb = forward + strafe - rotate;

        double max = Math.max(
                Math.abs(lf),
                Math.max(Math.abs(rf),
                        Math.max(Math.abs(lb), Math.abs(rb)))
        );

        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }
}
