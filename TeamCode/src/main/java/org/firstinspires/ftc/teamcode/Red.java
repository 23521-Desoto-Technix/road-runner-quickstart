package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import android.graphics.Bitmap;
import android.graphics.Canvas;
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
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import java.util.concurrent.atomic.AtomicReference;


@Autonomous(name="Red Backstage")
public class Red extends LinearOpMode {
    private FirstPipelineRevised firstPipelineRevised; //Create an object of the VisionProcessor Class
    private VisionPortal portal;
    private Servo claw = null;
    IMU imu;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        final CameraStreamProcessor processor = new CameraStreamProcessor();
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
                .addProcessor(processor)
                .setCameraResolution(new Size(1280, 720))
                .build();
        FtcDashboard.getInstance().startCameraStream(processor, 0);
        claw = hardwareMap.get(Servo.class, "claw");
        ModernRoboticsI2cRangeSensor rangeSensor;
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        boolean do_yellow = true;
        claw.setPosition(0.65);
        while (opModeInInit()) {
            telemetry.addLine(String.valueOf(firstPipelineRevised.getSelection()));
            telemetry.addData("Yellow? ", do_yellow);
            telemetry.update();
            if (gamepad1.dpad_right) {
                do_yellow = true;
            }
            if (gamepad1.dpad_left) {
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
                        .splineTo(new Vector2d(27, -3), Math.toRadians(-45))
                        .build();
                drive.followTrajectory(traj);
                if (do_yellow) {
                    traj = drive.trajectoryBuilder(traj.end(), true)
                            //.back(2)
                            //.splineTo(new Vector2d(20, -3), Math.toRadians(135))
                            .splineTo(new Vector2d(22, -37), Math.toRadians(-90))
                            .build();
                    drive.followTrajectory(traj);
                    //drive.turn(Math.toRadians(90) - drive.getRawExternalHeading());
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
                            .back(2)
                            .splineTo(new Vector2d(27, -3), Math.toRadians(-90))
                            .splineTo(new Vector2d(27, -36), Math.toRadians(-90))
                            .build();
                    drive.followTrajectory(traj);

                    drive.turn(Math.toRadians(90) - drive.getRawExternalHeading());
                } else {
                    traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .back(2)
                            .build();
                    drive.followTrajectory(traj);
                }
            } if (selection == 1) {
                traj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(12.5, 0), 0)
                        .splineTo(new Vector2d(27, 4), Math.toRadians(45))
                        .build();
                drive.followTrajectory(traj);
                if (do_yellow) {
                    traj = drive.trajectoryBuilder(traj.end(), true)
                            //.back(5)
                            //.splineTo(new Vector2d(30, -3), Math.toRadians(-90))
                            .splineTo(new Vector2d(34, -27), Math.toRadians(-90))
                            .build();
                    drive.followTrajectory(traj);
                    telemetry.addData("Heading", drive.getRawExternalHeading());
                    telemetry.update();
                    drive.turn(Math.toRadians(90) - drive.getRawExternalHeading());
                    traj = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                            .back(9)
                            .build();
                    drive.followTrajectory(traj);
                    traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(new Vector2d(33, -25), Math.toRadians(-90))
                            .splineTo(new Vector2d(33, -35), Math.toRadians(-90))
                            .build();
                } else {
                    traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .back(2)
                            .build();
                    drive.followTrajectory(traj);
                }
                //drive.followTrajectory(traj);
            }
            if (do_yellow) {
                leftArm.setTargetPosition(537);
                rightArm.setTargetPosition(537);
                leftArm.setPower(1);
                rightArm.setPower(1);
                while (leftArm.getCurrentPosition() < 537) {
                }
                claw.setPosition(0.5);
                sleep(100);
                leftArm.setTargetPosition(150);
                rightArm.setTargetPosition(150);
                while (leftArm.getCurrentPosition() > 250) {
                }
                stop();
            }
        }



    }
}
