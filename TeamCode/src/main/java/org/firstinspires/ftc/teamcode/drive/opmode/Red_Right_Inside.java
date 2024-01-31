package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class Red_Right_Inside extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo armLeft = hardwareMap.servo.get("armLeft");
        Servo armRight = hardwareMap.servo.get("armRight");
        Servo gripper = hardwareMap.servo.get("gripper");
        armLeft.setPosition(.05);
        armRight.setPosition(.95);
        sleep(2000);
        gripper.setPosition(.75);

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(11.5, -60, Math.toRadians(90)));

        TrajectorySequence RedRightInside_ToBoard = drive.trajectorySequenceBuilder(new Pose2d(11.50, -60.00, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(23.50, -13.50), Math.toRadians(90.00))
                .splineTo(new Vector2d(51.00, -42.50), Math.toRadians(0.00))
                .build();
        drive.setPoseEstimate(RedRightInside_ToBoard.start());

        TrajectorySequence RedRightInside_ToPark = drive.trajectorySequenceBuilder(new Pose2d(50.79, -42.50, Math.toRadians(0.00)))
                .lineToSplineHeading(new Pose2d(46.50, -16.50, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(63.00, -14.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();


        drive.followTrajectorySequence(RedRightInside_ToBoard);
        drive.followTrajectorySequence(RedRightInside_ToPark);
    }
}