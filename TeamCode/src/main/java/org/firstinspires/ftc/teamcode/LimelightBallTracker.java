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

@TeleOp(name="Limelight Ball Tracker", group="Limelight")
public class LimelightBallTracker extends LinearOpMode {

    // Drive motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Limelight
    private Limelight3A limelight;

    // Tracking parameters
    private static final double CENTER_TOLERANCE = 5.0; // degrees
    private static final double MIN_AREA = 0.5; // Minimum target area percentage
    private static final double MAX_DRIVE_POWER = 0.6;
    private static final double MIN_DRIVE_POWER = 0.15;
    
    // PID-like constants for tracking
    private static final double ROTATION_GAIN = 0.02; // How aggressively to turn toward target
    private static final double FORWARD_GAIN = 0.01; // How aggressively to drive toward target
    private static final double STRAFE_GAIN = 0.015; // How aggressively to strafe to center target

    // Target distance (in neural detector area percentage - larger = closer)
    private static final double TARGET_AREA = 5.0;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");

        // Set motor directions (adjust based on your robot)
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Start polling for data
        limelight.pipelineSwitch(0); // Switch to pipeline 0 (your neural detector pipeline)
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press START to begin ball tracking");
        telemetry.addLine("Press A to toggle between tracking and manual control");
        telemetry.update();

        waitForStart();

        boolean trackingMode = true;

        while (opModeIsActive()) {
            // Toggle tracking mode with A button
            if (gamepad1.a) {
                trackingMode = !trackingMode;
                sleep(200); // Debounce
            }

            if (trackingMode) {
                // Autonomous ball tracking
                trackBall();
            } else {
                // Manual control
                manualDrive();
            }

            telemetry.addData("Mode", trackingMode ? "TRACKING" : "MANUAL");
            telemetry.update();
        }

        // Stop limelight when done
        limelight.stop();
    }

    /**
     * Track and follow the ball using Limelight neural detection
     */
    private void trackBall() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Get detector results
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            if (detections != null && !detections.isEmpty()) {
                // Find the largest detection (closest/most prominent ball)
                LLResultTypes.DetectorResult largestDetection = null;
                double largestArea = 0;

                for (LLResultTypes.DetectorResult detection : detections) {
                    double area = detection.getTargetArea();
                    if (area > largestArea) {
                        largestArea = area;
                        largestDetection = detection;
                    }
                }

                if (largestDetection != null && largestArea > MIN_AREA) {
                    // Get ball position relative to crosshair
                    double tx = largestDetection.getTargetXDegrees(); // Horizontal offset
                    double ty = largestDetection.getTargetYDegrees(); // Vertical offset
                    double area = largestDetection.getTargetArea();

                    // Calculate motor powers based on ball position
                    double rotate = tx * ROTATION_GAIN;
                    double forward = (TARGET_AREA - area) * FORWARD_GAIN;
                    double strafe = tx * STRAFE_GAIN;

                    // Clamp values
                    rotate = Range.clip(rotate, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
                    forward = Range.clip(forward, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
                    strafe = Range.clip(strafe, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

                    // Apply minimum power threshold
                    if (Math.abs(rotate) < MIN_DRIVE_POWER && Math.abs(rotate) > 0) {
                        rotate = Math.signum(rotate) * MIN_DRIVE_POWER;
                    }
                    if (Math.abs(forward) < MIN_DRIVE_POWER && Math.abs(forward) > 0) {
                        forward = Math.signum(forward) * MIN_DRIVE_POWER;
                    }

                    // Stop if centered and at target distance
                    if (Math.abs(tx) < CENTER_TOLERANCE && Math.abs(area - TARGET_AREA) < 1.0) {
                        stopDrive();
                        telemetry.addLine("✓ BALL CENTERED");
                    } else {
                        // Drive toward ball
                        mecanumDrive(forward, strafe, rotate);
                    }

                    // Telemetry
                    telemetry.addData("Ball Found", "YES");
                    telemetry.addData("X Offset", "%.2f°", tx);
                    telemetry.addData("Y Offset", "%.2f°", ty);
                    telemetry.addData("Area", "%.2f%%", area);
                    telemetry.addData("Class", largestDetection.getClassName());
                    telemetry.addData("Confidence", "%.2f%%", largestDetection.getConfidence() * 100);
                    telemetry.addData("Forward", "%.2f", forward);
                    telemetry.addData("Strafe", "%.2f", strafe);
                    telemetry.addData("Rotate", "%.2f", rotate);
                } else {
                    // Ball too small or not found
                    stopDrive();
                    telemetry.addData("Ball Found", "TOO FAR");
                }
            } else {
                // No detections
                stopDrive();
                telemetry.addData("Ball Found", "NO");
                telemetry.addLine("Searching...");
            }
        } else {
            // No valid result
            stopDrive();
            telemetry.addData("Limelight", "No Data");
        }
    }

    /**
     * Manual drive control using gamepad
     */
    private void manualDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        mecanumDrive(forward, strafe, rotate);

        telemetry.addLine("Use left stick to move, right stick to turn");
    }

    /**
     * Mecanum drive control
     */
    private void mecanumDrive(double forward, double strafe, double rotate) {
        double leftFrontPower = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower = forward - strafe + rotate;
        double rightBackPower = forward + strafe - rotate;

        // Normalize powers
        double maxPower = Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(rightFrontPower),
                        Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Stop all drive motors
     */
    private void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
