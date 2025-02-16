package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;

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

    // distance from the center of the Limelight lens to the floor
    final double CAMERA_HEIGHT = 10.5;
    // how many degrees back is your limelight rotated from perfectly vertical?
    final double CAMERA_ANGLE = 0;
    // Since your target is on the floor, this is 0
    final double TARGET_HEIGHT = 0;

    // Desired stop distance from the target (in inches)
    final double STOP_DISTANCE = 30;


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

            // Extract the last character and convert it to an integer
            char lastChar = limelightName.charAt(limelightName.length() - 1);
            if (Character.isDigit(lastChar)) {
                int pipelineIndex = Character.getNumericValue(lastChar);
                telemetry.addData("Found Limelight: ", limelightName + " using Pipeline: " + pipelineIndex);
                limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
                limelight.pipelineSwitch(pipelineIndex);
                limelight.start();
            } else {
                telemetry.addData("Error", "Invalid Limelight name format: " + limelightName);
            }
        } else {
            telemetry.addData("Error", "No active Limelight found");
        }
        telemetry.update();

        waitForStart();
        telemetry.addData("x", 0);
        telemetry.addData("y", 0);
        telemetry.update();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();  // How far left or right the target is (degrees)
                double ty = result.getTy();  // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                // Convert ty (vertical offset) to distance
                double adjustedAngle = Math.toRadians(CAMERA_ANGLE + ty);
                if (Math.abs(adjustedAngle) > 0) { // Prevent division by zero
                    targetDistance = Math.abs((CAMERA_HEIGHT - TARGET_HEIGHT) / Math.tan(adjustedAngle));
                } else {
                    targetDistance = Double.POSITIVE_INFINITY; // Prevent division by zero
                }
                // Is The Data Fresh?
                telemetry.addData("Data", result.getStaleness());
                //  Color Results
                List<ColorResult> colorTargets = result.getColorResults();
                for (ColorResult colorTarget : colorTargets) {
                    double x = colorTarget.getTargetXDegrees(); // Where it is (left-right)
                    double y = colorTarget.getTargetYDegrees(); // Where it is (up-down)
                    double area = colorTarget.getTargetArea(); // size (0-100)
                    telemetry.addData("Color Target", String.format("x: %.4f, y: %.4f, takes up %.2f%% of the image", x, y, area));
                }

                telemetry.addData("Offset", String.format("tx: %.2f, ty: %.2f, ta: %.2f", tx, ty, ta));
                telemetry.addData("Adjusted Angle", String.format("%.2f", adjustedAngle));
                telemetry.addData("Distance", String.format("from Target: %.2f inches, stop at: %.2f inches", targetDistance, STOP_DISTANCE));
                telemetry.update();


                // Stop when the robot is within the stop distance
                if (targetDistance <= STOP_DISTANCE) {
                    stopMotors();
//                    limelight.stop(); // Turn off camera to stop tracking
//                    break;
                }
                double distanceError = targetDistance - STOP_DISTANCE; // Distance error (positive means too far, negative means too close)
                double forwardPower = -0.03 * distanceError; // Move forward/backward based on error
                double turnPower = -0.035 * tx; // Negative to correct direction
                drive(forwardPower, turnPower);

            } else {
//                telemetry.addData(":(", "not detected yet....");
//                telemetry.update();
                stopMotors(); // Stop if no target detected
            }
        }
    }

    private void drive(double forward, double turn) {
        double leftPower = forward + turn;
        double rightPower = forward - turn;

        leftFrontMotor.setPower(leftPower);
        rightFrontMotor.setPower(rightPower);
        leftBackMotor.setPower(leftPower);
        rightBackMotor.setPower(rightPower);
    }

    private void stopMotors() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

}
