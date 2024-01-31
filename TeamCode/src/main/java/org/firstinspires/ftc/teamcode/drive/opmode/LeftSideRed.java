package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class LeftSideRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo armLeft = hardwareMap.servo.get("armLeft");
        Servo armRight = hardwareMap.servo.get("armRight");
        Servo gripper = hardwareMap.servo.get("gripper");
        armLeft.setPosition(.05);
        armRight.setPosition(.95);
        sleep(2000);
        gripper.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(-36.00, -64.00, Math.toRadians(90.00)));

        Trajectory dropoff = drive.trajectoryBuilder(new Pose2d(-36.00, -64.00, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(-47.53, -23.95), Math.toRadians(90.00))
                .build();

        TrajectorySequence toBoard = drive.trajectorySequenceBuilder(new Pose2d(-47.53, -23.95, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(-24.40, -34.78), Math.toRadians(90.00))
                .lineToLinearHeading(new Pose2d(33.44, -34.78, Math.toRadians(90.00)))
                .splineTo(new Vector2d(51.98, -29.59), Math.toRadians(0.00))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(51.98, -29.59, Math.toRadians(360.00)))
                .lineToLinearHeading(new Pose2d(48.57, -11.79, Math.toRadians(360.00)))
                .splineTo(new Vector2d(63.55, -13.87), Math.toRadians(360.00))
                .build();

        drive.followTrajectory(dropoff);
        gripper.setPosition(1);
        armLeft.setPosition(.1);
        armRight.setPosition(.9);
        gripper.setPosition(0);
        drive.followTrajectorySequence(toBoard);
        armLeft.setPosition(.8);
        armRight.setPosition(.2);
        drive.followTrajectorySequence(park);
    }
}
