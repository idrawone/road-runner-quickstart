package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Limelight Drive Auto", group = "Autonomous")
public class test extends LinearOpMode {

    public static double CAMERA_HEIGHT = 16.0;       // inches; height of the Limelight from the floor
    public static double TARGET_HEIGHT = 1.5;      // inches; height of the vision target
    public static double CAMERA_ANGLE_DEG = 75.0;     // degrees; Limelight mounting angle

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize the Limelight3A (ensure "limelight" matches your robot configuration)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Set the pipeline to your color-detection pipeline (for example, pipeline 8)
        limelight.pipelineSwitch(8);
        limelight.start();

        // Initialize your Road Runner drive system.
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // --- Continuously search for a valid target until one is detected ---
        LLResult result = null;
        while (opModeIsActive()) {
            result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("Target", "Detected");
                telemetry.update();
                break;
            } else {
                telemetry.addData("Target", "Not Detected");
                telemetry.update();
            }
            idle();
        }

        if (result == null || !result.isValid()) {
            telemetry.addData("Error", "No valid target detected. Exiting.");
            telemetry.update();
            return;
        }

        // --- Compute distance to the target using Limelight's vertical offset (ty) ---
        double ty = result.getTy();  // vertical offset in degrees
        double cameraAngleRad = Math.toRadians(CAMERA_ANGLE_DEG);
        double tyRad = Math.toRadians(ty);
        // Compute the distance using:
        // distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / tan(CAMERA_ANGLE_DEG + ty)
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(cameraAngleRad + tyRad);

        telemetry.addData("ty (deg)", ty);
        telemetry.addData("Distance (in)", distance);
        telemetry.update();

        // --- Build and execute the trajectory using Road Runner ---
        // Here, we drive straight forward along the Y-axis by the computed distance.
        // For a robot starting at (0,0,0) with heading 0 (facing positive Y),
        // this means the target position is at (0, distance).
        Action driveToTarget = drive.actionBuilder(initialPose)
                .lineToX(distance)
                .build();

        // Execute the trajectory (this call blocks until complete).
        Actions.runBlocking(driveToTarget);

        telemetry.addData("Status", "Reached target position");
        telemetry.update();
        limelight.stop();
    }
}
