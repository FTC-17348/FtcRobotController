package org.firstinspires.ftc.teamcode.comp.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.shared.GlobalConfig;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig.ALLIANCE_POS;
import org.firstinspires.ftc.teamcode.shared.MotionHardware;
import org.firstinspires.ftc.teamcode.shared.MotionHardware.Direction;
import org.firstinspires.ftc.teamcode.shared.VisionHardware;
import org.firstinspires.ftc.teamcode.shared.VisionHardware.PropPosition;

@Config
@Autonomous(name = "Auto - BA Right", group = "Auto")
public class AutoBR extends LinearOpMode {

    public ALLIANCE_POS alliancePos = ALLIANCE_POS.RIGHT;

    public GlobalConfig globalConfig = new GlobalConfig(GlobalConfig.AUTONOMOUS_DELIVERY_MODES.DROPPER);
    MotionHardware robot = new MotionHardware(this, globalConfig);
    VisionHardware vision = new VisionHardware(this, alliancePos);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        vision.init();

        robot.moveArm(.5, 5, 5);

        waitForStart();

        //TODO Add Park from BR

        // Center Strike Line
        // - Forward: 31.75
        // - Reverse: -44.75
        while(opModeIsActive()) {
            PropPosition propPosition = vision.detectProp();

            switch(propPosition) {
                case MIDDLE:
                    // Dropper Mode
                    robot.moveRobot(.5, -24, 10);
                    robot.dropPixel();
                    robot.moveArmMotorToPosition(0, 10);
                    break;
                case RIGHT:
                    //Dropper Mode
                    robot.moveRobot(.5, -16, 10);
                    robot.turnRobot(Direction.RIGHT, 6, .5, 10);
                    robot.moveRobot(.5, -4, 10);
                    robot.dropPixel();
                    robot.moveArmMotorToPosition(0, 10);
                    break;
                default:
                    //Dropper Mode
                    robot.moveRobot(.9, -16, 10);
                    robot.turnRobot(Direction.LEFT, 10, .8, 10);
                    robot.moveRobot(.9, -8, 10);
                    robot.dropPixel();
                    robot.moveArmMotorToPosition(0, 10);
                    break;
            }
            
            sleep(20);
            break;
        }
    }
}
