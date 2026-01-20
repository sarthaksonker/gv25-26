package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.OTOSKt;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class OTOSLocalizer implements Localizer {

    public static class Params {
        public double angularScalar = 1.0;
        public double linearScalar = 1.0;

        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
    }

    public static Params PARAMS = new Params();

    private final SparkFunOTOS otos;
    private Pose2d currentPose;

    // 🔥 Add IMU for heading
    private final BNO055IMU imu;
    private double imuOffset = 0;

    public OTOSLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        currentPose = initialPose;

        otos.setPosition(OTOSKt.toOTOSPose(currentPose));
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.calibrateImu();
        otos.setLinearScalar(PARAMS.linearScalar);
        otos.setAngularScalar(PARAMS.angularScalar);
        otos.setOffset(PARAMS.offset);

        // 🔥 Setup IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters p = new BNO055IMU.Parameters();
        p.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(p);

        imuOffset = initialPose.heading.toDouble();
    }

    // ✅ Getter for OTOS so other classes can access it
    public SparkFunOTOS getOtos() {
        return otos;
    }

    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        currentPose = pose;

        otos.setPosition(OTOSKt.toOTOSPose(currentPose));

        imuOffset = pose.heading.toDouble() - imu.getAngularOrientation().firstAngle;
    }

    @Override
    public PoseVelocity2d update() {

        // get OTOS XY data
        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose, otosVel, otosAcc);

        // 🔥 Use IMU heading (instant, no lag)
        double imuHeading = imu.getAngularOrientation().firstAngle + imuOffset;

        // use OTOS X/Y + IMU heading
        currentPose = new Pose2d(
                otosPose.x,
                otosPose.y,
                imuHeading
        );

        // velocities
        Vector2d fieldVel = new Vector2d(otosVel.x, otosVel.y);
        Vector2d robotVel = Rotation2d.exp(imuHeading).inverse().times(fieldVel);

        return new PoseVelocity2d(robotVel, otosVel.h);
    }
}
