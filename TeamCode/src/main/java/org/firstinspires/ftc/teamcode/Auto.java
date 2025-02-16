//package org.firstinspires.ftc.teamcode;
//
//import androidx.annotation.NonNull;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Config
//@Autonomous
//public class Auto extends LinearOpMode {
//
//    // Inner class for controlling the chain (arm) motor.
//    public class Chain {
//        private DcMotorEx chainMotor;
//
//        public Chain(HardwareMap hardwareMap) {
//            chainMotor = hardwareMap.get(DcMotorEx.class, "chain");
//            chainMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            chainMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//
//        // Action to run the chain motor to a specific encoder position.
//        public class RunToPosition implements Action {
//            private int target;
//            private boolean initialized = false;
//
//            public RunToPosition(int target) {
//                this.target = target;
//            }
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    chainMotor.setTargetPosition(target);
//                    chainMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    chainMotor.setPower(0.5); // Adjust power as needed.
//                    initialized = true;
//                }
//                int currentPos = chainMotor.getCurrentPosition();
//                packet.put("chainPos", currentPos);
//                // Continue running until the current position is within 10 counts of target.
//                if (Math.abs(currentPos - target) > 10) {
//                    return true;
//                } else {
//                    chainMotor.setPower(0);
//                    return false;
//                }
//            }
//        }
//
//        public Action runToPosition(int target) {
//            return new RunToPosition(target);
//        }
//
//        // Slow movement of chain to lower position.
//        public Action runToPositionSlow(int target, double power) {
//            return new Action() {
//                private boolean initialized = false;
//
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    if (!initialized) {
//                        chainMotor.setTargetPosition(target);
//                        chainMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                        chainMotor.setPower(power); // Reduced power for smooth descent.
//                        initialized = true;
//                    }
//                    int currentPos = chainMotor.getCurrentPosition();
//                    packet.put("chainPos", currentPos);
//                    if (Math.abs(currentPos - target) > 10) {
//                        return true;
//                    } else {
//                        chainMotor.setPower(0);
//                        return false;
//                    }
//                }
//            };
//        }
//    }
//
//    // Inner class for controlling servos.
//    public class MyServos {
//        private Servo servo1, servo2;
//        private CRServo crServo;
//
//        public MyServos(HardwareMap hardwareMap) {
//            servo1 = hardwareMap.get(Servo.class, "wrist");
//            servo2 = hardwareMap.get(Servo.class, "intake");
//            crServo = hardwareMap.get(CRServo.class, "intake2");
//        }
//
//        // Action to set servo positions.
//        public class SetPositions implements Action {
//            private double pos1, pos2, power;
//
//            public SetPositions(double pos1, double pos2, double power) {
//                this.pos1 = pos1;
//                this.pos2 = pos2;
//                this.power = power;
//            }
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                servo1.setPosition(pos1);
//                servo2.setPosition(pos2);
//                crServo.setPower(power);
//                return false;
//            }
//        }
//
//        public Action setPositions(double pos1, double pos2, double power) {
//            return new SetPositions(pos1, pos2, power);
//        }
//
//        // Action to spin CRServo for a duration.
//        public Action spinCRServoFor(double power, double duration) {
//            return new Action() {
//                private long startTime = -1;
//
//                @Override
//                public boolean run(@NonNull TelemetryPacket packet) {
//                    if (startTime == -1) {
//                        startTime = System.currentTimeMillis();
//                        crServo.setPower(power);
//                    }
//                    long elapsed = System.currentTimeMillis() - startTime;
//                    packet.put("crServoPower", power);
//                    if (elapsed < duration * 1000) {
//                        return true;
//                    } else {
//                        crServo.setPower(0);
//                        return false;
//                    }
//                }
//            };
//        }
//    }
//
//    @Override
//    public void runOpMode() {
//        // Define the starting pose.
//        Pose2d initialPose = new Pose2d(0, 0, 0);
//
//        // Initialize the drive system.
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//
//        // Initialize the chain motor and servos.
//        Chain chain = new Chain(hardwareMap);
//        MyServos servos = new MyServos(hardwareMap);
//
//        // Wait for the start button.
//        waitForStart();
//        if (isStopRequested()) return;
//
//        // 1. Set wrist servo to 0.675 before any movement.
//        Action setWristAction = servos.setPositions(0.675, 0, 0);
//
//        // 2. Drive along a spline to (48, 52) with heading π/2.
//        Action driveAction = drive.actionBuilder(initialPose)
//                .splineTo(new Vector2d(48, 52), Math.PI / 2)
//                .build();
//
//        // 3. Raise the chain quickly to encoder count 300.
//        Action chainAction = chain.runToPosition(300);
//
//        // 4. Drive slowly for 7 inches while spinning the CR servo for 2 seconds.
//        Pose2d slowDriveStart = new Pose2d(48, 52, Math.PI / 2);
//        Action slowDriveAction = drive.actionBuilder(slowDriveStart)
//                .lineToY(41)  // Move 7 inches slowly
//                .build();
//
//        Action spinCRServoTimed = servos.spinCRServoFor(0.5, 2.0);
//        Action parallelSlowDriveWithSpin = new ParallelAction(slowDriveAction, spinCRServoTimed);
//
//        // 5. Lower the chain back to 0 slowly, only AFTER the CR servo stops.
//        Action chainDownAction = chain.runToPositionSlow(0, 0.3);
//
//        // Execute all actions in sequence.
//        Actions.runBlocking(new SequentialAction(
//                setWristAction,          // Set wrist servo to 0.675
//                driveAction,             // Drive to (48,52)
//                chainAction,             // Raise chain to 300
//                parallelSlowDriveWithSpin, // Slow drive while CR servo spins
//                chainDownAction          // Lower chain slowly to 0
//        ));
//    }
//}
