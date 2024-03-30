package org.firstinspires.ftc.teamcode.kuykendall;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SharedCode {
    // Motors
    public DcMotor leftLinearExtension = null;
    public DcMotor rightLinearExtension = null;
    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor intakeMotor = null;

    // Servos
    public Servo intakeLiftServo = null;
    public Servo bucketServo = null;
    public Servo bucketWristServo = null;
    public Servo armWristServo = null;
    public Servo planeServo = null;

    // Constants for servo positions
    final double INTAKE_LIFT_DOWN = 0.2;
    final double INTAKE_LIFT_UP = 0.4;
    final double BUCKET_UP = 0.3;
    final double BUCKET_DROP_1 = 0.1;
    final double BUCKET_DROP_2 = 0.5;
    final double WRIST_INTAKE = 0.1;
    final double WRIST_DELIVERY = 0.4;
    final double PLANE_HOLD = 0.2;
    final double PLANE_ACTIVE = 0.6;

    // Intake toggle state
    private boolean intakeOn = false;

    HardwareMap hwMap = null;

    public SharedCode() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Initialize motors
        leftLinearExtension = hwMap.get(DcMotor.class, "leftLinearExtension");
        rightLinearExtension = hwMap.get(DcMotor.class, "rightLinearExtension");
        frontLeftDrive = hwMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hwMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hwMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hwMap.get(DcMotor.class, "backRightDrive");
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");

        // Set zero power behavior for linear extension motors to BRAKE
        leftLinearExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize servos
        intakeLiftServo = hwMap.get(Servo.class, "intakeLiftServo");
        bucketServo = hwMap.get(Servo.class, "bucketServo");
        bucketWristServo = hwMap.get(Servo.class, "bucketWristServo");
        armWristServo = hwMap.get(Servo.class, "armWristServo");
        planeServo = hwMap.get(Servo.class, "planeServo");

        // Set motor and servo initial positions
        planeServo.setPosition(PLANE_HOLD);
    }

    // Function to control linear extension motors
    public void controlLinearExtension(Gamepad gamepad) {
        if (gamepad.y) {
            leftLinearExtension.setPower(1);
            rightLinearExtension.setPower(1);
        } else if (gamepad.a) {
            leftLinearExtension.setPower(-1);
            rightLinearExtension.setPower(-1);
        } else {
            // Motors automatically brake due to zero power behavior set to BRAKE
            leftLinearExtension.setPower(0);
            rightLinearExtension.setPower(0);
        }
    }


    // Drive functions
    public void drive(Gamepad gamepad) {
        double drive = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double rotate = gamepad.right_stick_x;

        double flPower = drive + strafe + rotate;
        double frPower = drive - strafe - rotate;
        double blPower = drive - strafe + rotate;
        double brPower = drive + strafe - rotate;

        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
    }

    // Function to toggle intake motor
    public void toggleIntake(Gamepad gamepad) {
        if (gamepad.x && !intakeOn) {
            intakeMotor.setPower(-1);
            intakeOn = true;
        } else if (gamepad.x && intakeOn) {
            intakeMotor.setPower(0);
            intakeOn = false;
        }
    }

    // Servo control functions
    public void controlServos(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            intakeLiftServo.setPosition(INTAKE_LIFT_DOWN);
        } else if (gamepad.left_bumper) {
            intakeLiftServo.setPosition(INTAKE_LIFT_UP);
        }

        if (gamepad.dpad_up) {
            bucketServo.setPosition(BUCKET_UP);
        } else if (gamepad.dpad_left) {
            bucketServo.setPosition(BUCKET_DROP_1);
        } else if (gamepad.dpad_right) {
            bucketServo.setPosition(BUCKET_DROP_2);
        }

        if (gamepad.y) {
            bucketWristServo.setPosition(WRIST_DELIVERY);
            armWristServo.setPosition(WRIST_DELIVERY);
        } else if (gamepad.a) {
            bucketWristServo.setPosition(WRIST_INTAKE);
            armWristServo.setPosition(WRIST_INTAKE);
        }

        if (gamepad.dpad_down) {
            planeServo.setPosition(PLANE_ACTIVE);
        } else {
            planeServo.setPosition(PLANE_HOLD);
        }
    }
}
