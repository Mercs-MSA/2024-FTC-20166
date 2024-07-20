package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name="Testing Road Runner", group="")
public class AutonTest extends LinearOpMode {
    public void runOpMode() {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

     // We want to start the bot at x: 10, y: -8, heading: 90 degrees
    Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

      drive.setPoseEstimate(startPose);
      Trajectory traj1 = drive.trajectoryBuilder(startPose)
        .splineTo(new Vector2d(20, 9), Math.toRadians(45))
        .build();

      Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
         .splineTo(new Vector2d(20, 9), Math.toRadians(45))
        .build();

  drive.followTrajectory(traj1);
  drive.followTrajectory(traj2);
}
    }
}
