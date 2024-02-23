package org.firstinspires.ftc.teamcode.driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.shared.MotionHardware;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;


@TeleOp(name = "Drive", group = "TeleOp2Driver")
public class Driver extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV = -1.0;     // Maximum REV power applied to motor

    static final double ARM_SPEED = 1.0;
    static final int ARM_DROP_POS_LOW = -4520;
    static final int ARM_DROP_POS_HIGH = -3540;
    static final int ARM_DRIVE_POS = -830;
    static final int ARM_INTAKE_POS = 25;
    static final double LEFT_GRIPPER_OPEN = 0.90;
    //was 1.5
    static final double LEFT_GRIPPER_CLOSE = 2.2;
    static final double RIGHT_GRIPPER_OPEN = 0.1;

    //was -0.5
    static final double RIGHT_GRIPPER_CLOSE = -1.1;
    static final double WRIST_DROP_POS_LOW = 0.85;
    static final double WRIST_DROP_POS_HIGH = 01;
    static final double WRIST_INTAKE_POS = 0.3;
    static final double WRIST_FORWARD_DROP_POS_HIGH = 0.62; //was 73
    static final double WRIST_FORWARD_DROP_POS_LOW = 0.65;
    static final double DRONE_LAUNCH = 0.8;
    static final double DRONE_SECURE = 0.3;
    private Servo leftGripper;
    private Servo rightGripper;
    private Servo wristServo;
    private Servo launcherServo = null;
    private DcMotor armMotor = null;

    private Thread launcherThread;
    private Thread armThread;
    MotionHardware robot = new MotionHardware(this);


    @Override
    public void runOpMode() {

        // Define class members
        DcMotor motor;
        double power = 0;
        boolean rampUp = true;
        //Define variables for arm, wrist, and gripper
        //DcMotor armMotor;
        //CRServo contServo;
        float leftY, rightY;
        double armPosition, gripPosition, contPower;
        double MIN_POSITION = 0, MAX_POSITION = 1;
        // called when init button is  pressed.


        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        //armMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wristServo = hardwareMap.servo.get("wristServo");
        launcherServo = hardwareMap.get(Servo.class, "planeServo");
        //DroneCoverServo = hardwareMap.get(Servo.class, "DroneCoverServo");
        leftGripper = hardwareMap.get(Servo.class, "leftGripper");
        rightGripper = hardwareMap.get(Servo.class, "rightGripper");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //robot.init();

        initializeLauncherThread();
        initializeArmThread();

        waitForStart();

        launcherThread.start();
        armThread.start();




        telemetry.addData("Mode", "waiting");

        while (opModeIsActive()) {
            telemetry.addData("Mode", "running");
            // check to see if we need to move the servo.

            telemetry.addData("Servo Position", launcherServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

            telemetry.addData("Mode", "running");
            // check to see if we need to move the servo.

            //telemetry.addData("Servo Position", DroneCoverServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

            // check arm position
            telemetry.addData("Arm Position",armMotor.getCurrentPosition());
            telemetry.update();

            // Declare Motors
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator; //
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);



            if (gamepad2.right_bumper) {
                // Move servos in opposite directions when "y" is pressed
                leftGripper.setPosition(LEFT_GRIPPER_CLOSE);
                rightGripper.setPosition(RIGHT_GRIPPER_CLOSE);// Ad'just the position value as needed

            } else if (gamepad2.left_bumper) {
                // Return servos to the center position when "x" is pressed
                leftGripper.setPosition(LEFT_GRIPPER_OPEN);
                rightGripper.setPosition(RIGHT_GRIPPER_OPEN); // Adjust the position value for the center position
            }
            else if (gamepad1.x) {
                // Move servos in opposite directions when "y" is pressed
                leftGripper.setPosition(LEFT_GRIPPER_CLOSE);
                rightGripper.setPosition(RIGHT_GRIPPER_CLOSE);// Adjust the position value as needed

            } else if (gamepad1.y) {
                // Return servos to the center position when "x" is pressed
                leftGripper.setPosition(LEFT_GRIPPER_OPEN);
                rightGripper.setPosition(RIGHT_GRIPPER_OPEN); // Adjust the position value for the center position
            }
            telemetry.update();
        }

        // Stop the threads when the op mode is no longer active
        if (launcherThread != null) launcherThread.interrupt();
        if (armThread != null) armThread.interrupt();
    }




    private void initializeLauncherThread() {
        launcherThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.interrupted()) {
                   /* if (gamepad2.dpad_left) {
                        // move to 0 degrees.
                        DroneCoverServo.setPosition(0.3);
                    } else if (gamepad2.dpad_right) {
                        // move to 90 degrees.
                        DroneCoverServo.setPosition(.7);
                    }

                    if (gamepad2.a) {
                        // move to 0 degrees.
                        launcherServo.setPosition(0.3);
                    } else if (gamepad2.b || gamepad2.a) {
                        // move to 90 degrees.
                        launcherServo.setPosition(1);
                    }*/

                    /*
                    if (gamepad1.dpad_left) {
                        // move to 0 degrees.
                        DroneCoverServo.setPosition(0.3);
                    } else if (gamepad1.dpad_right) {
                        // move to 90 degrees.
                        DroneCoverServo.setPosition(.7);
                    }*/


                    // Launch Drone
                    //TODO Validate the servo positions for launch
                    if (gamepad1.left_bumper) {
                        // Launch Drone
                        launcherServo.setPosition(DRONE_LAUNCH);
                    } else if (gamepad1.right_bumper) {
                        // Secure Drone
                        launcherServo.setPosition(DRONE_SECURE);
                    }
                     }
                    try {
                        Thread.sleep(10); // Small delay to prevent looping too fast
                    } catch (InterruptedException e) {

                }
            }
        });
    }


    private void initializeArmThread() {
        ElapsedTime runtime = new ElapsedTime();

        armThread = new Thread(new Runnable() {
            @Override
            public void run() {
                float leftY, rightY;
                while (!Thread.interrupted()) {
                    //leftY = gamepad2.left_stick_y * -1;
                    //rightY = gamepad2.right_stick_y * -1;
                    //armMotor.setPower(Range.clip(leftY, -1.0, 1.0));
                    //Between here..


                    if (gamepad2.y) {
                        wristServo.setPosition(WRIST_DROP_POS_HIGH);
                        moveArmMotorToPosition(ARM_DROP_POS_HIGH, 2);
                    }
                    else if (gamepad2.dpad_down) {
                        wristServo.setPosition(WRIST_INTAKE_POS);
                        moveArmMotorToPosition(ARM_INTAKE_POS, 2.6);
                    } else if (gamepad2.dpad_up) {
                        wristServo.setPosition(WRIST_INTAKE_POS);
                        moveArmMotorToPosition(ARM_DRIVE_POS, 2.6);
                    }else if (gamepad2.x) {
                        wristServo.setPosition(WRIST_DROP_POS_LOW);
                        //moveArmMotorToPosition(ARM_DROP_POS_LOW, 2);
                        moveArmMotorToPositionProgBrake(ARM_DROP_POS_LOW, 2);
                    }else if (gamepad1.b) {
                        // move to 0 degrees.
                        wristServo.setPosition(WRIST_FORWARD_DROP_POS_HIGH);
                        moveArmMotorToPosition(ARM_DRIVE_POS, 2.6);
                    }else if (gamepad1.a) {
                        wristServo.setPosition(WRIST_INTAKE_POS);
                        moveArmMotorToPosition(ARM_INTAKE_POS, 2.6);
                    }
                   /* if (gamepad1.dpad_up) {
                        armMotor.setTargetPosition(-4185);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        runtime.reset();
                        armMotor.setPower(Math.abs(1));

                        armMotor.setPower(0);

                        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    }
                    else if (gamepad1.dpad_down){
                        armMotor.setTargetPosition(1);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        runtime.reset();
                        armMotor.setPower(Math.abs(1));

                        armMotor.setPower(0);

                        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    }
                    */
                    leftY = gamepad1.left_trigger * -1;
                    rightY = gamepad1.right_trigger * 1;
                    armMotor.setPower(Range.clip(leftY, -1.0, 1.0));
                    armMotor.setPower(Range.clip(rightY, -1.0, 1.0));
                    //and here put your logic to move the arm up and down
                    try {
                        Thread.sleep(10); // Small delay to prevent looping too fast
                    } catch (InterruptedException e) {
                        break; // Exit the loop if the thread is interrupted
                    }
                }
            }
        });
    }

    private void moveArmMotorToPosition(int position, double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();

        runtime.reset();
        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_SPEED); // Set your desired power
        while ((armMotor.isBusy()) && (runtime.seconds() < timeoutS)) {

            telemetry.addData("Running to", "%7d", position);
            telemetry.addData("Currently at", "%7d", armMotor.getCurrentPosition());
            telemetry.update();
        }
        armMotor.setPower(0); // Stop the motor once the position is reached
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void moveArmMotorToPositionProgBrake(int position, double timeoutS) {
        ElapsedTime runtime = new ElapsedTime();
        int currPosition = armMotor.getCurrentPosition();
        int progBrakeStartPosition = position < currPosition ?
                (int)(position + ((currPosition - position)*.1)) :  //moving below currPosition
                (int)(position - ((position - currPosition)*.1));   //moving above currPosition

        HashMap<Integer, Double> brakeMap = new HashMap<Integer, Double>();

        int stepIncrement = position < currPosition ?
                (int)(((currPosition - position)*.1)/5) :
                (int)(((position - currPosition)*.1)/5);

        int brakePos1 = position < currPosition ?
                position + (stepIncrement*5) :
                position - (stepIncrement*5);
        brakeMap.put(brakePos1, 0.95);
        int brakePos2 = position < currPosition ?
                position + (stepIncrement*4) :
                position - (stepIncrement*4);
        brakeMap.put(brakePos2, 0.90);
        int brakePos3 = position < currPosition ?
                position + (stepIncrement*3) :
                position - (stepIncrement*3);
        brakeMap.put(brakePos3, 0.85);
        int brakePos4 = position < currPosition ?
                position + (stepIncrement*2) :
                position - (stepIncrement*2);
        brakeMap.put(brakePos4, 0.80);


        runtime.reset();
        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_SPEED); // Set your desired power
        while ((armMotor.isBusy()) && (runtime.seconds() < timeoutS)) {

            if(position < currPosition) {
                if (armMotor.getCurrentPosition() < progBrakeStartPosition) {
                    int currPos = armMotor.getCurrentPosition();
                    AtomicInteger distance = new AtomicInteger(10000);
                    AtomicInteger idx = new AtomicInteger();
                    brakeMap.forEach((key, value) -> {
                        int cdistance = Math.abs(key - currPos);
                        if(cdistance < distance.get()) {
                            idx.set(key);
                            distance.set(cdistance);
                        }
                    });
                    armMotor.setPower(brakeMap.get(idx));
                }
            } else {
                if (armMotor.getCurrentPosition() > progBrakeStartPosition) {
                    int currPos = armMotor.getCurrentPosition();
                    AtomicInteger distance = new AtomicInteger(10000);
                    AtomicInteger idx = new AtomicInteger();
                    brakeMap.forEach((key, value) -> {
                        int cdistance = Math.abs(key - currPos);
                        if(cdistance < distance.get()) {
                            idx.set(key);
                            distance.set(cdistance);
                        }
                    });
                    armMotor.setPower(brakeMap.get(idx));
                }
            }
            telemetry.addData("Running to", "%7d", position);
            telemetry.addData("Currently at", "%7d", armMotor.getCurrentPosition());
            telemetry.update();
        }
        armMotor.setPower(0); // Stop the motor once the position is reached
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}



