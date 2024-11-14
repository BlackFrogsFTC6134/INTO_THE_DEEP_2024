
package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "Near Net Zone", group = "Test")
public class NrNetZn_Auton extends LinearOpMode {
    private Servo continuousIntakeServo1;
    private DcMotorEx linearViper = null;
    private DcMotorEx rotateViper = null;
    double linearViperPower = 0;
    double rotateViperPower = 0;
    public static int RT_targetPosition = 3500;
    public static int LI_targetPosition = 450;
    public static int RT_targetPositionRung =4000;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        linearViper = hardwareMap.get(DcMotorEx.class, "Linear_Viper");
        rotateViper = hardwareMap.get(DcMotorEx.class, "Rotate_Viper");

        linearViper.setPower(linearViperPower);
        rotateViper.setPower(rotateViperPower);

        linearViper.setDirection(DcMotorEx.Direction.REVERSE);
        rotateViper.setDirection(DcMotorEx.Direction.REVERSE);

        linearViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotateViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        linearViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rotateViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //linearViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        linearViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotateViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //LINEAR_VIPER_TARGET_POSITION_DOWN = linearViper.getCurrentPosition();
        //ROTATE_VIPER_TARGET_POSITION_DOWN = rotateViper.getCurrentPosition();

        //telemetry.addData(">> Linear viper position (ticks/rev) ", linearViper.getCurrentPosition());
        //telemetry.addData(">> Linear viper position (ticks/rev) ", rotateViper.getCurrentPosition());

        telemetry.addLine(">> linear & rotate vipers: Initialized");
        telemetry.update();

        // Define the starting pose of the robot
        Pose2d startPose = new Pose2d(0, 0, 0);

        // Create a trajectory to move forward by 24 inches
        //TrajectoryActionBuilder tab1 = drive.actionBuilder(drive.pose)
        //      .lineToY(24);


        // Initialize claws. Additional configuration needed.
        continuousIntakeServo1 = hardwareMap.get(Servo.class, "clawServo");

        // Set initial positions

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        if (opModeIsActive()) {

            continuousIntakeServo1.setPosition(0.00);

            sleep(100);
            rotateViper.setTargetPosition(RT_targetPosition);
            rotateViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rotateViper.setPower(0.9);

            while (rotateViper.isBusy()) {
                telemetry.addData("Current position", rotateViper.getTargetPosition());
                telemetry.update();
            }
            rotateViper.setPower(0);

            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .strafeTo(new Vector2d(28, 10))
                            .build()

            );
            sleep(100);
            linearViper.setTargetPosition(LI_targetPosition);
            linearViper.setPower(0.9);
            linearViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            while (linearViper.isBusy()) {
                telemetry.addData("Current position", linearViper.getTargetPosition());
                telemetry.update();
            }
            linearViper.setPower(0);
            sleep(100);
            continuousIntakeServo1.setPosition(0.35);
            sleep(1000);

            //TO push the spike sample to the observation zone
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(26.5, 25))
                            .waitSeconds(1)
                            .lineToX(60)
                            .waitSeconds(2)
                            .splineTo(new Vector2d(60,34), Math.toRadians(180))
                            .waitSeconds(2)
                            .lineToX(20)
                            .turnTo(Math.toRadians(45))
                            .strafeTo(new Vector2d(10, 40))
                            .waitSeconds(2)
                            .splineTo(new Vector2d(10,34), Math.toRadians(160))
                     //       .turnTo(Math.toRadians(45))
                            .lineToX(60)
                            .turnTo(Math.toRadians(90))
                            .lineToY(15)
                            .build()
            );
            sleep(100);
            rotateViper.setTargetPosition(RT_targetPositionRung);
            rotateViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rotateViper.setPower(0.9);

            while (rotateViper.isBusy()) {
                telemetry.addData("Current position", rotateViper.getTargetPosition());
                telemetry.update();
            }
            rotateViper.setPower(0);
            sleep(100);
        }
    }
}