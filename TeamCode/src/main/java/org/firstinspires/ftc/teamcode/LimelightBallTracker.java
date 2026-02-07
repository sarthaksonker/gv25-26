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

@TeleOp(name = "Limelight Aim Assist (Fixed 2.0)", group = "Limelight")
public class LimelightBallTracker extends LinearOpMode {

    // Drive motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Limelight
    private Limelight3A limelight;

    // ===== TUNING =====
    private static final double ROTATION_GAIN = 0.05;  // increase to make robot respond faster
    private static final double MAX_ROTATE_POWER = 0.45;
    private static final double MIN_ROTATE_POWER = 0.10;
    private static final double TX_DEADBAND = 1.5;      // degrees
    private static final long TARGET_TIMEOUT_MS = 400; // longer to smooth target loss

    // Target hold
    private double lastTx = 0;
    private long lastSeenTime = 0;

    private boolean aimAssistEnabled = true;

    @Override
    public void runOpMode() {

        // MOTOR SETUP
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "lb");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rb");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Aim Assist: A = OFF | B = ON");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // TOGGLE AIM ASSIST
            if (gamepad1.a) {
                aimAssistEnabled = false;
                sleep(250);
            }
            if (gamepad1.b) {
                aimAssistEnabled = true;
                sleep(250);
            }

            // DRIVER INPUT
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double manualRotate = gamepad1.right_stick_x;

            double autoRotate = 0;

            // ===== AIM ASSIST =====
            if (aimAssistEnabled && Math.abs(manualRotate) < 0.15) { // slightly bigger deadzone

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

                    if (detections != null && !detections.isEmpty()) {
                        LLResultTypes.DetectorResult best = null;
                        double bestArea = 0;

                        for (LLResultTypes.DetectorResult d : detections) {
                            if (d.getConfidence() < 0.6) continue;

                            double area = d.getTargetArea();
                            if (area > bestArea) {
                                bestArea = area;
                                best = d;
                            }
                        }

                        if (best != null) {
                            lastTx = best.getTargetXDegrees();
                            lastSeenTime = System.currentTimeMillis();
                        }
                    }
                }

                // Use last seen target briefly
                long now = System.currentTimeMillis();
                if (now - lastSeenTime < TARGET_TIMEOUT_MS) {

                    if (Math.abs(lastTx) > TX_DEADBAND) {
                        autoRotate = lastTx * ROTATION_GAIN; // remove minus if robot rotates wrong way

                        autoRotate = Range.clip(autoRotate, -MAX_ROTATE_POWER, MAX_ROTATE_POWER);

                        if (Math.abs(lastTx) > 5 && Math.abs(autoRotate) < MIN_ROTATE_POWER) {
                            autoRotate = Math.signum(autoRotate) * MIN_ROTATE_POWER;
                        }
                    }
                }
            }

            double finalRotate = manualRotate + autoRotate;
            mecanumDrive(forward, strafe, finalRotate);

            // TELEMETRY
            telemetry.addData("Aim Assist", aimAssistEnabled);
            telemetry.addData("Manual Rotate", manualRotate);
            telemetry.addData("Auto Rotate", autoRotate);
            telemetry.addData("Last TX", lastTx);
            telemetry.addData("Target Age (ms)", System.currentTimeMillis() - lastSeenTime);
            telemetry.update();
        }

        limelight.stop();
    }

    private void mecanumDrive(double forward, double strafe, double rotate) {
        double lf = forward + strafe + rotate;
        double rf = forward - strafe - rotate;
        double lb = forward - strafe + rotate;
        double rb = forward + strafe - rotate;

        double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf),
                Math.max(Math.abs(lb), Math.abs(rb))));

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
