package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
@Autonomous(name = "Trejectory_Test_Net", group = "Test")
public class Trejectory_Test_Net extends LinearOpMode {

   @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Define the starting pose of the robot
        Pose2d startPose = new Pose2d(0, 0, 0);

        // To reach the basket
        Action TrajectoryAction1 = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(10,26.5), Math.toRadians(90))
              //  .strafeTo(new Vector2d(10, 26.5))
                .waitSeconds(1)
                .build();
        // To reach the spike sample
        Action TrajectoryAction2 =  drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(10,26.5), Math.toRadians(90))
                .waitSeconds(1)
                .lineToX(10)
                .build();
        //TO reach the basket again to drop the sample
        Action TrajectoryAction3 =  drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(20,26.5), Math.toRadians(190))
                .lineToX(-10)
                .waitSeconds(1)
                .build();
        //To reach the spike sample to take the 2nd spike sample
        Action TrajectoryAction4 =  drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(10,26.5), Math.toRadians(190))
                .waitSeconds(1)
                .lineToX(10)
                .build();
       //To reach the basket to drop the 2nd spike sample
       Action TrajectoryAction5 =  drive.actionBuilder(drive.pose)
               .splineTo(new Vector2d(20,26.5), Math.toRadians(190))
               .lineToX(-10)
               .waitSeconds(1)
               .build();
       //To reach 3rd spike sample
       Action TrajectoryAction6 =  drive.actionBuilder(drive.pose)
               .splineTo(new Vector2d(10,26.5), Math.toRadians(190))
               .waitSeconds(1)
               .lineToX(50)
               .build();
       //To push  3rd spike sample into the net zone
       Action TrajectoryAction7 =  drive.actionBuilder(drive.pose)
               .splineTo(new Vector2d(60,26.5), Math.toRadians(190))
               .waitSeconds(1)
               .lineToX(-50)
               .build();

        waitForStart();
        if (isStopRequested()) return;

        if (opModeIsActive()) {

            sleep(100);
            Actions.runBlocking(
                    new SequentialAction(
                            TrajectoryAction1,
                            TrajectoryAction2,
                            TrajectoryAction3,
                            TrajectoryAction4,
                            TrajectoryAction5,
                            TrajectoryAction6,
                            TrajectoryAction7
                    )
            );
            sleep(100);
        }
    }
}

