package org.firstinspires.ftc.teamcode.swift;
import static android.os.SystemClock.sleep;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig;
import org.firstinspires.ftc.teamcode.shared.MotionHardware_Pixel_Dropper_Test_DS_DONT_USE_FOR_AUTON;

@TeleOp(name = "Pixel_Dropper_Test",group = "TeleOp2Driver")
public class Pixel_Dropper_Test extends LinearOpMode{
    public GlobalConfig globalConfig = new GlobalConfig(GlobalConfig.AUTONOMOUS_DELIVERY_MODES.DROPPER);
    MotionHardware_Pixel_Dropper_Test_DS_DONT_USE_FOR_AUTON robot = new MotionHardware_Pixel_Dropper_Test_DS_DONT_USE_FOR_AUTON(this, globalConfig);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        waitForStart();
        robot.dropPixel();
    }
}
