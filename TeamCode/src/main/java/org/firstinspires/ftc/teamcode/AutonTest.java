package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//This autonomous is for testing Road Runner
@Autonomous (name="Testing Road Runner", group="")
public class AutonTest extends LinearOpMode {
    MecanumDrive drive;
    private void initializeMotors() {
        drive.leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    double StartPoseX = -5.0;
    double StartPoseY = -5.0;
    double StartPoseT = -45.0;
    @Override
    public void runOpMode() {
        Pose2d StartPose = new Pose2d(StartPoseX, StartPoseY, Math.toRadians(StartPoseT));
        //We are following this: https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/ while working on this. Setting starting pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose);
        //initializeMotors();

        Action trajectoryAction1;

        trajectoryAction1 = drive.actionBuilder(drive.pose)
//                .strafeTo(new Vector2d(10, -5))
                .splineTo(new Vector2d(30, 30), Math.toRadians(72))
                .build();
        waitForStart();
        //while (opModeIsActive()) {
            Actions.runBlocking(trajectoryAction1);
        //}
    }
}
