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
@Autonomous(name = "Auto - BA Left No Back", group = "Auto")
public class AutoBLJS extends LinearOpMode {

    public ALLIANCE_POS alliancePos = ALLIANCE_POS.LEFT;

    public GlobalConfig globalConfig = new GlobalConfig(GlobalConfig.AUTONOMOUS_DELIVERY_MODES.DROPPER);
    MotionHardware robot = new MotionHardware(this, globalConfig);
    VisionHardware vision = new VisionHardware(this, alliancePos);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        vision.init();

        robot.moveArm(.5, 5, 5);

        waitForStart();

        //TODO In parking code, drop arms to TeleOp pickup position

        // Center Strike Line
        // - Forward: 31.75
        // - Reverse: -44.75
        while(opModeIsActive()) {
            PropPosition propPosition = vision.detectProp();

            switch(propPosition) {
                case MIDDLE:
                    robot.dropperDown();
                    robot.moveRobot(.7, -24, 10);
                    robot.dropperUp();
                    break;
                case LEFT:
                    robot.dropperDown();
                    robot.moveRobot(.75, -24, 2);
                    robot.turnRobot(Direction.LEFT, 18.5, .5, 2);
                    robot.moveRobot(.5, 2, 2);
                    robot.dropperUp();
                    break;
                default:
                    //Dropper Mode
                    robot.dropperDown();
                    robot.moveRobot(.6, -24, 2);
                    robot.turnRobot(Direction.RIGHT, 18.5, .5, 2);
                    robot.moveRobot(.75,-6,2);
                    robot.dropperUp();
                    break;
            }

            sleep(20);
            break;
        }
    }
}
