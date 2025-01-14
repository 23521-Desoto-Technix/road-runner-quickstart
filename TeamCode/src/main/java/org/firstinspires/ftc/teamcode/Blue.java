package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import android.util.Size;
import org.firstinspires.ftc.teamcode.FirstPipelineRevised;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
@Autonomous(name="Blue Backstage")
public class Blue extends LinearOpMode {
    private FirstPipelineRevised firstPipelineRevised; //Create an object of the VisionProcessor Class
    private VisionPortal portal;
    private Servo claw = null;
    IMU imu;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    @Override
    public void runOpMode() throws InterruptedException {
        firstPipelineRevised = new FirstPipelineRevised();
        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setTargetPosition(0);
        rightArm.setTargetPosition(0);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(firstPipelineRevised)
                .setCameraResolution(new Size(1280, 720))
                .build();
        claw = hardwareMap.get(Servo.class, "claw");
        ModernRoboticsI2cRangeSensor rangeSensor;
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        boolean do_yellow = true;
        claw.setPosition(0.7);
        while (opModeInInit()) {
            telemetry.addLine(String.valueOf(firstPipelineRevised.getSelection()));
            telemetry.addData("Yellow? ", do_yellow);
            telemetry.update();
            if (gamepad1.dpad_left) {
                do_yellow = true;
            }
            if (gamepad1.dpad_right) {
                do_yellow = false;
            }
        }
        waitForStart();
        portal.setProcessorEnabled(firstPipelineRevised, true);
        if (opModeIsActive()) {
            telemetry.addLine(String.valueOf(firstPipelineRevised.getSelection()));
            telemetry.update();
            Trajectory traj = null;
            double selection = firstPipelineRevised.getSelection();
            if (selection == 3) {
                traj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(12.5, 0), 0)
                        .splineTo(new Vector2d(25, -3.5), Math.toRadians(-45))
                        .build();
                drive.followTrajectory(traj);
                if (do_yellow) {
                    traj = drive.trajectoryBuilder(traj.end(), true)
                            .back(2)
                            .splineTo(new Vector2d(33.5, 26), Math.toRadians(90))
                            .build();
                    drive.followTrajectory(traj);
                    drive.turn(Math.toRadians(-90) - drive.getRawExternalHeading());
                    traj = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                            .back(10)
                            .build();
                    drive.followTrajectory(traj);
                } else {
                    traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .back(2)
                            .build();
                    drive.followTrajectory(traj);
                }
            } if (selection == 2) {
                traj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(28.5, 0), 0)
                        .build();
                drive.followTrajectory(traj);
                if (do_yellow) {
                    traj = drive.trajectoryBuilder(traj.end(), true)
                            .back(1)
                            //.splineTo(new Vector2d(26.5, 3), Math.toRadians(90))
                            .splineTo(new Vector2d(26.5, 36), Math.toRadians(90))
                            .build();
                    drive.followTrajectory(traj);
                    drive.turn(Math.toRadians(-90) - drive.getRawExternalHeading());
                } else {
                    traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .back(2)
                            .build();
                    drive.followTrajectory(traj);
                }
                /*
                drive.turn(Math.toRadians(-90));
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(37)
                        .build();
                drive.followTrajectory(traj);*/

            } if (selection == 1) {
                traj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(12.5, 0), 0)
                        .splineTo(new Vector2d(27, 4), Math.toRadians(45))
                        .build();
                drive.followTrajectory(traj);
                if (do_yellow) {
                    traj = drive.trajectoryBuilder(traj.end(), true)
                            .back(2)
                            //.splineTo(new Vector2d(30, -3), Math.toRadians(-90))
                            //.splineTo(new Vector2d(22, 35), Math.toRadians(90))
                            .build();
                    drive.followTrajectory(traj);
                    drive.turn(Math.toRadians(-145.5));
                    traj = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                            .splineTo(new Vector2d(22, 37), Math.toRadians(90))
                            .build();
                    drive.followTrajectory(traj);
                    drive.turn(Math.toRadians(-90) - drive.getRawExternalHeading());
                } else {
                    traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .back(2)
                            .build();
                    drive.followTrajectory(traj);
                }
            }
            if (do_yellow) {
                leftArm.setTargetPosition(567);
                rightArm.setTargetPosition(567);
                leftArm.setPower(1);
                rightArm.setPower(1);
                while (leftArm.getCurrentPosition() < 537 && !isStopRequested()) {
                }
                claw.setPosition(0.5);
                sleep(100);
                leftArm.setTargetPosition(250);
                rightArm.setTargetPosition(250);
                while (leftArm.getCurrentPosition() > 250 && !isStopRequested()) {
                }
            }
            traj = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                    .splineTo(new Vector2d(22, -31), Math.toRadians(270))
                    .build();
            //drive.followTrajectory(traj);
        }



    }
}
