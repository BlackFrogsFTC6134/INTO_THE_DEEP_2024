package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

// https://gist.github.com/TheOutcastVirus/fc0de0afdcb37808288904e308a67dc7

@Autonomous(name = "Trejectory_Test", group = "Test")
public class Trejectory_Test extends LinearOpMode {
     @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Define the starting pose of the robot
        Pose2d startPose = new Pose2d(0, 0, 0);

        // To reach the submersible
        Action TrajectoryAction1 = drive.actionBuilder(startPose)
                                .strafeTo(new Vector2d(26.5, 10))
                                .waitSeconds(1)
                                .build();
        //TO push the spike sample to the observation zone
        Action TrajectoryAction2 =  drive.actionBuilder(drive.pose)
                                   .strafeTo(new Vector2d(26.5, -27))
                                   .waitSeconds(1)
                                   .lineToX(60)
                                   .waitSeconds(1)
                                   .splineTo(new Vector2d(60,-34), Math.toRadians(190))
                                   .waitSeconds(2)
                                   .lineToX(10)
                                   .strafeTo(new Vector2d(13, -14))
                                   .turnTo(Math.toRadians(30))
                                   .waitSeconds(5)
                                   .build();

        //TO move away from observation zone and then move to submersible
        Action TrajectoryAction3 =  drive.actionBuilder(drive.pose)
                                    //.turnTo(Math.toRadians(180))
                                    .waitSeconds(1)
                                    .strafeTo(new Vector2d(26.5, 10))
        //        .splineTo(new Vector2d(10, -14), Math.toRadians(180))
   //             .splineTo(new Vector2d(10,-34), Math.toRadians(90))
//                                     .splineTo(new Vector2d(10,-34), Math.toRadians(90))
 //                                    .lineToY(10)
 //                                    .waitSeconds(1)
 //                                    .strafeTo(new Vector2d(26.5, 10))

                                     .build();
        //Move to the Observation zone
        Action TrajectoryAction4 =  drive.actionBuilder(drive.pose)
                               .strafeTo(new Vector2d(10, 10))
                                    .waitSeconds(1)
                                    .build();
        waitForStart();
        if (isStopRequested()) return;

        if (opModeIsActive()) {

   //         continuousIntakeServo1.setPosition(0.00);
            sleep(100);
            Actions.runBlocking(
                    new SequentialAction(
                            TrajectoryAction1,
                            TrajectoryAction2,
                            TrajectoryAction3,
                            TrajectoryAction4
                    )
            );
            sleep(100);
/*
            //Strafe to the right
            Action TrajectoryAction3 = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(-13, 20), Math.toRadians(-90))
                    .build();


            //Moves to the right
            Action TrajectoryAction4 = drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(0, -10))
                    .build();

            //Goes forward
            Action TrajectoryAction6 = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(20, 0), Math.toRadians(90))
                    .build();

            //Gets the robot ready to push the sample into the observation zone
            Action TrajectoryAction7 = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(35, 2.5), Math.toRadians(180))
                    .build();

            //Pushes minimum 2 samples in Observation zone
            Action TrajectoryAction8 = drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(35, 0))
                    .build();

            //Gets ready to push another sample into the observation zone
            Action TrajectoryAction9 = drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(0, 10))
                    .build();

            //Parks in the observation zone while pushing the next sample inside
            Action TrajectoryAction10 = drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(35, 0))
                    .build();

             */
        }
    }
}
