package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Autonomous(name = "Dual VisionPortals")
public class insane extends LinearOpMode {

    VisionPortal.Builder myVisionPortalBuilder;
    boolean USE_WEBCAM_1;
    int Portal_1_View_ID;
    boolean USE_WEBCAM_2;
    int Portal_2_View_ID;
    AprilTagProcessor myAprilTagProcessor_2;
    private FirstPipelineRevised firstPipelineRevised;
    VisionPortal myVisionPortal_1;
    VisionPortal myVisionPortal_2;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo claw = null;

    /**
     * Describe this function...
     */
    private void initMultiPortals() {
        List myPortalsList;

        myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false)).intValue();
        Portal_2_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false)).intValue();
        telemetry.addData("Portal 1 View ID (index 0 of myPortalsList)", Portal_1_View_ID);
        telemetry.addData("Portal 2 View ID (index 1 of myPortalsList)", Portal_2_View_ID);
        telemetry.addLine("");
        telemetry.addLine("Press Y to continue");
        telemetry.update();
        while (!gamepad1.y && opModeInInit()) {
            // Loop until gamepad Y button is pressed.
        }
    }

    /**
     * This Op Mode demonstrates MultiPortalView.
     *
     * The Dpad buttons can turn each camera stream on and off.
     * USB bandwidth is more restricted on an external USB hub, compared to the Control Hub.
     */
    @Override
    public void runOpMode() {
        // This OpMode shows AprilTag recognition and pose estimation.
        USE_WEBCAM_1 = true;
        USE_WEBCAM_2 = true;
        initMultiPortals();
        // Initialize AprilTag before waitForStart.
        initAprilTag();
        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
        claw = hardwareMap.get(Servo.class, "claw");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setTargetPosition(0);
        rightArm.setTargetPosition(0);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Wait for the Start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        boolean do_yellow = true;
        setManualExposure(1, 250);
        if (opModeIsActive()) {
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
        myVisionPortal_1.setProcessorEnabled(firstPipelineRevised, true);
        if (opModeIsActive()) {
            telemetry.addLine(String.valueOf(firstPipelineRevised.getSelection()));
            telemetry.update();
            Trajectory traj = null;
            double selection = firstPipelineRevised.getSelection();
            myVisionPortal_1.close();
            if (selection == 3) {
                traj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(12.5, 0), 0)
                        .splineTo(new Vector2d(27, -3), Math.toRadians(-45))
                        .build();
                drive.followTrajectory(traj);

                if (do_yellow) {
                    traj = drive.trajectoryBuilder(traj.end(), true)
                            .back(3)
                            .build();
                    drive.followTrajectory(traj);
                    drive.turn(Math.toRadians(135));
                    straighten(6);
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
                            .splineTo(new Vector2d(35, -27), Math.toRadians(-90))
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
                            .splineTo(new Vector2d(35, -25), Math.toRadians(-90))
                            .splineTo(new Vector2d(35, -35), Math.toRadians(-90))
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

    /**
     * Initialize AprilTag Detection.
     */
    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;

        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Create each AprilTagProcessor by calling build.
        myAprilTagProcessor_2 = myAprilTagProcessorBuilder.build();
        Make_first_VisionPortal();
        Make_second_VisionPortal();
    }

    /**
     * Describe this function...
     */
    private void Make_first_VisionPortal() {
        // Create a VisionPortal.Builder and set attributes related to the first camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM_1) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Manage USB bandwidth of two camera streams, by adjusting resolution from default 640x480.
        // Set the camera resolution.
        // Manage USB bandwidth of two camera streams, by selecting Streaming Format.
        // Set the stream format.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        firstPipelineRevised = new FirstPipelineRevised();
        myVisionPortalBuilder.addProcessor(firstPipelineRevised);
        myVisionPortalBuilder.setCameraResolution(new Size(1280, 720));
        // Add the Portal View ID to the VisionPortal.Builder
        // Set the camera monitor view id.
        myVisionPortalBuilder.setLiveViewContainerId(Portal_1_View_ID);
        // Create a VisionPortal by calling build.
        myVisionPortal_1 = myVisionPortalBuilder.build();
    }

    /**
     * Describe this function...
     */
    private void Make_second_VisionPortal() {
        if (USE_WEBCAM_2) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Manage USB bandwidth of two camera streams, by adjusting resolution from default 640x480.
        // Set the camera resolution.
        // Manage USB bandwidth of two camera streams, by selecting Streaming Format.
        // Set the stream format.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor_2);
        // Add the Portal View ID to the VisionPortal.Builder
        // Set the camera monitor view id.
        myVisionPortalBuilder.setLiveViewContainerId(Portal_2_View_ID);
        myVisionPortalBuilder.setCameraResolution(new Size(1920, 1080));
        // Create a VisionPortal by calling build.
        myVisionPortal_2 = myVisionPortalBuilder.build();
    }

    /**
     * Describe this function...
     */
    private void Toggle_camera_streams() {
        // Manage USB bandwidth of two camera streams, by turning on or off.
        if (gamepad1.dpad_down) {
            // Temporarily stop the streaming session. This can save CPU
            // resources, with the ability to resume quickly when needed.
            myVisionPortal_1.stopStreaming();
        } else if (gamepad1.dpad_up) {
            // Resume the streaming session if previously stopped.
            myVisionPortal_1.resumeStreaming();
        }
        if (gamepad1.dpad_left) {
            // Temporarily stop the streaming session. This can save CPU
            // resources, with the ability to resume quickly when needed.
            myVisionPortal_2.stopStreaming();
        } else if (gamepad1.dpad_right) {
            // Resume the streaming session if previously stopped.
            myVisionPortal_2.resumeStreaming();
        }
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private void AprilTag_telemetry_for_Portal_1() {
        telemetry.addData("Prop location", String.valueOf(firstPipelineRevised.getSelection()));
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private void AprilTag_telemetry_for_Portal_2() {
        List<AprilTagDetection> myAprilTagDetections_2;
        AprilTagDetection thisDetection_2;

        // Get a list of AprilTag detections.
        myAprilTagDetections_2 = myAprilTagProcessor_2.getFreshDetections();
        
        // Iterate through list and call a function to display info for each recognized AprilTag.
        if (myAprilTagDetections_2 != null) {
            telemetry.addLine("");
            telemetry.addData("Portal 2 - # AprilTags Detected", JavaUtil.listLength(myAprilTagDetections_2));
            for (AprilTagDetection thisDetection_2_item : myAprilTagDetections_2) {
                thisDetection_2 = thisDetection_2_item;
                // Display info about the detection.
                telemetry.addLine("");
                if (thisDetection_2.metadata != null) {
                    telemetry.addLine("==== (ID " + thisDetection_2.id + ") " + thisDetection_2.metadata.name);
                    telemetry.addLine("XYZ " + JavaUtil.formatNumber(thisDetection_2.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.z, 6, 1) + "  (inch)");
                    telemetry.addLine("PRY " + JavaUtil.formatNumber(thisDetection_2.ftcPose.yaw, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.roll, 6, 1) + "  (deg)");
                    telemetry.addLine("RBE " + JavaUtil.formatNumber(thisDetection_2.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
                } else {
                    telemetry.addLine("==== (ID " + thisDetection_2.id + ") Unknown");
                    telemetry.addLine("Center " + JavaUtil.formatNumber(thisDetection_2.center.x, 6, 0) + "" + JavaUtil.formatNumber(thisDetection_2.center.y, 6, 0) + " (pixels)");
                }
            }
        }
    }

    /**
     * Describe this function...
     */
    private void AprilTag_telemetry_legend() {
        telemetry.addLine("");
        telemetry.addLine("key:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    private void straighten(int tag_id) {
        boolean targetFound = false;
        AprilTagDetection desiredTag  = null;
        final double SPEED_GAIN =   0.1;
        final double TURN_GAIN  =   -0.02 ;

        final double MAX_AUTO_SPEED = 0.75;
        final double MAX_AUTO_TURN  = 0.25;
        // Step through the list of detected tags and look for a matching tag
        double rangeError = 10000;
        double headingError = 10000;
        while ((rangeError > 0.5 || headingError > 0.5) && opModeIsActive()) {
            targetFound = false;
            desiredTag = null;
            List<AprilTagDetection> currentDetections = myAprilTagProcessor_2.getFreshDetections();
            while (!targetFound && opModeIsActive()) {
                if (currentDetections != null) {
                    for (AprilTagDetection detection : currentDetections) {
                        // Look to see if we have size info on this tag.
                        if (detection.metadata != null) {
                            //  Check to see if we want to track towards this tag.
                            //telemetry.addData("desired", tag_id);
                            //telemetry.addData("tag", detection.id);
                            if (detection.id == tag_id) {
                                // Yes, we want to use this tag.
                                targetFound = true;
                                desiredTag = detection;
                                break;  // don't look any further.
                            } else {
                                // This tag is in the library, but we do not want to track it right now.
                                //telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        } else {
                            // This tag is NOT in the library, so we don't have enough information to track to it.
                            //telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        }
    
                    }
                    
                }
                AprilTag_telemetry_for_Portal_2();
                telemetry.update();
            }
            telemetry.addData("found",targetFound);
            telemetry.addData("detection", desiredTag);
            if (targetFound) {
                rangeError = (desiredTag.ftcPose.range-8);
                headingError = desiredTag.ftcPose.x;

                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                moveRobot(-drive, -turn);
                telemetry.addData("Error", "range %5.2f, heading %5.2f", rangeError, headingError);
                telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
                telemetry.update();
            } else {
                moveRobot(0,0);
            }
            sleep(10)
        }
        moveRobot(0,0);
    }
    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max >1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Send powers to the wheels.
        //telemetry.addData("left", leftPower);
        //telemetry.addData("right", rightPower);
        //telemetry.update();
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (myVisionPortal_2 == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (myVisionPortal_2.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (myVisionPortal_2.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = myVisionPortal_2.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = myVisionPortal_2.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }
}
