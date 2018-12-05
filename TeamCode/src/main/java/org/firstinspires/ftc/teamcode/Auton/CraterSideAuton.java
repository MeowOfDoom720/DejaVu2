/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.Auton;

import android.os.WorkSource;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc.teamcode.dependencies.Robot;


/**
 * CraterSideAuton
 * The CraterSideAuton program is the code and instructions for
 * the robot based on starting on the Crater side of the lander.
 *
 * @author Gaurav
 * @version 1.0
 * @since 2018-10-29
 */

@Autonomous(name="Crater Side Auton", group="Internal")
//@Disabled
public class CraterSideAuton extends LinearOpMode {
    /**
     * This class extends the "Robot" class, a dependencies
     * class that holds motor and rotational control
     */
    Robot r = new Robot(this);

    boolean left = false;
    boolean middle = false;
    boolean right = false;
    //TODO better names
    String directionForMineralFirst = "";
    String directionForMineralSecond = "";
    double distanceForLineUp = 0;

    double baseDistance = 30;

    double inchesToDepot = 60;
    double inchesToCrater = 70;

    String samplePos = "center";

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        r.start(hardwareMap, telemetry);
        r.init();


        //r.detectorInit();
        //for center
        //24 inches backward
        //6 inches forward
        //rotate 90 ccw
        //drive backwards 43 in
        //rotate 135 ccw
        //39 inches backward
        //60 +/-3 inches forward
        //drop arm

        //for left
        //rotate 26 ccw
        //25.5 inches backwards
        //rotate 105 ccw
        //33 inches backwards
        //rotate 135 ccw
        //37 inches backwards
        //61 inches forward

        //for right
        //rotate 26 cw
        //25.5 inches backwards
        //6 inches forward
        //rotate 90 ccw
        //50 inches backwards
        //rotate 135 ccw
        //40 inches backwards
        //59 inches forward

        waitForStart();
        while(opModeIsActive()){
            r.armMotors[0].setPower(-0.25);

            sleep(100);
            r.servoMotors[1].setPosition(0.35);
            sleep(100);

            r.positionDrive(2, -40, 0.2);
            sleep(100);


            //r.positionDrive(0,840,0.3 );

            r.armMotors[0].setPower(0.5);
            while(!isStopRequested() && r.armMotors[0].getCurrentPosition() <= 820) {}
            r.armMotors[0].setPower(0);

            r.rotate("cw",0.1,18);
            //TODO translate 4
            r.translate(4,-0.3);
            r.rotate("ccw",0.1,0);
            r.translate(32.5, -0.15);
            break;
        }

    }
}
