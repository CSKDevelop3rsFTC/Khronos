package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.742)
            .forwardZeroPowerAcceleration(-33.04)
            .lateralZeroPowerAcceleration(-55.18)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.095, 0, 0.0077, 0.02)) //p-0.1 d-0.01
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.03, 0.027))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.023,0.0,0.00018,0.6,0.01))
            .centripetalScaling(0.00045);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.48, 1.05);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) //Changing does not work for power
            .xVelocity(71.73)
            .yVelocity(52.45)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftFrontMotorName("fl")
            .leftRearMotorName("bl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.7)
            .strafePodX(-7.7)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}