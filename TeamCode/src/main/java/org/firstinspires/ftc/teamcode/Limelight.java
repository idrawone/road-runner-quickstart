package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.ArrayList;
import java.util.List;

@Autonomous
public class Limelight extends LinearOpMode {

    private DcMotorEx leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    private Limelight3A limelight;
    private double targetDistance = 0;

    // Constants for distance calculation
    final double CAMERA_HEIGHT = 10.5;  // Limelight height from the ground (adjust based on your robot)
    final double CAMERA_ANGLE = 0;   // Angle of the Limelight relative to the ground (in degrees)
    final double STOP_DISTANCE = 30;  // Desired stop distance from the target (in inches)


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Configure motor directions & modes
        leftFrontMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Get all active Limelight cameras from the hardware map
        List<Limelight3A> activeLimelights = new ArrayList<>(hardwareMap.getAll(Limelight3A.class));

        if (!activeLimelights.isEmpty()) {
            // Assuming only one Limelight is active, get its name
            limelight = activeLimelights.get(0);
            String limelightName = hardwareMap.getNamesOf(limelight).iterator().next();

            telemetry.addData("Active Limelight", limelightName);
            telemetry.update();

            // Extract the last character and convert it to an integer
            char lastChar = limelightName.charAt(limelightName.length() - 1);
            if (Character.isDigit(lastChar)) {
                int pipelineIndex = Character.getNumericValue(lastChar);
                telemetry.addData("Selected Pipeline", pipelineIndex);
                limelight.pipelineSwitch(pipelineIndex);
                limelight.start();
            } else {
                telemetry.addData("Error", "Invalid Limelight name format: " + limelightName);
            }
            telemetry.update();
        } else {
            telemetry.addData("Error", "No active Limelight found");
            telemetry.update();
        }
        waitForStart();
        telemetry.addData("x", 0);
        telemetry.addData("y", 0);
        telemetry.update();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double xOffset = result.getTx();  // Left/Right offset
                double yOffset = result.getTy();  // Forward/Backward offset

                // Convert ty (vertical offset) to distance
                double adjustedAngle = Math.toRadians(yOffset); // No need to subtract from camera angle since it's 0
                if (Math.abs(adjustedAngle) > 0) {
                    targetDistance = Math.abs(CAMERA_HEIGHT / Math.tan(adjustedAngle));
                } else {
                    targetDistance = Double.POSITIVE_INFINITY; // Prevent division by zero
                }
                telemetry.addData("xOffset", xOffset);
                telemetry.addData("yOffset", yOffset);
                telemetry.addData("Adjusted Angle", adjustedAngle);
                telemetry.addData("Distance to Target", targetDistance);
                telemetry.addData("stop distance", STOP_DISTANCE);
                telemetry.update();

                // Stop when the robot is within the stop distance
                if (targetDistance <= STOP_DISTANCE) {
                    stopMotors();
//                    limelight.stop(); // Turn off camera to stop tracking
//                    break;
                }
                double distanceError = targetDistance - STOP_DISTANCE; // Distance error (positive means too far, negative means too close)
                double forwardPower = -0.03 * distanceError; // Move forward/backward based on error

                // Ensure power is within a reasonable range (-0.5 to 0.5)
                forwardPower = Math.max(-0.5, Math.min(0.5, forwardPower));
                // Turning power to adjust for alignment
                double turnPower = -0.003 * xOffset;
                // Drive the robot
                drive(forwardPower, turnPower);

            } else {
//                telemetry.addData(":(", "not detected yet....");
//                telemetry.update();
                stopMotors(); // Stop if no target detected
            }
        }
    }

    private void drive(double forward, double turn) {
        leftFrontMotor.setPower(forward + turn);
        rightFrontMotor.setPower(forward - turn);
        leftBackMotor.setPower(forward + turn);
        rightBackMotor.setPower(forward - turn);
    }

    private void stopMotors() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}
