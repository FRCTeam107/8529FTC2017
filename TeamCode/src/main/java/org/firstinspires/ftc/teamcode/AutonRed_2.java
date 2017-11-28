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

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.TimeFormatException;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code assumes that you do NOT have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * The desired path in this example is:
 * - Drive forward for 3 seconds
 * - Spin right for 1.3 seconds
 * - Drive Backwards for 1 Second
 * - Stop and close the claw.
 * <p>
 * The code is written in a simple form with no optimizations.
 * However, there are several ways that this type of sequence could be streamlined,
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "AutonRed_2", group = "Pushbot")
public class AutonRed_2 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareBot robot = new HardwareBot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = .5;
    static final double TURN_SPEED = 0.75;
    static final double armServo = 0;


    VuforiaLocalizer vuforia;
    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 0.1;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    // Define class members
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//Move servo down
        while (opModeIsActive()) {
            // If possible, turn the light on in the beginning (it might already be on anyway,
            // we just make sure it is if we can).
            if (robot.colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) robot.colorSensor).enableLight(true);
            }

            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT;
                if (position >= MAX_POS) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }

//set color sensor up in hardwareMap
//                colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");


                // Read the sensor
                NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();


                // values is a reference to the hsvValues array.
                float[] hsvValues = new float[3];
                final float values[] = hsvValues;
//

                Color.colorToHSV(colors.toColor(), hsvValues);

                /** We also display a conversion of the colors to an equivalent Android color integer.
                 * @see Color */
                int color = colors.toColor();


                float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
                colors.red /= max;
                colors.green /= max;
                colors.blue /= max;
                color = colors.toColor();

                if (Color.blue(color) > 100) {
                    robot.leftDrive.setPower(-FORWARD_SPEED);
                    robot.rightDrive.setPower(-FORWARD_SPEED);
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < .3)) {
                        telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }
                } else {
                    robot.leftDrive.setPower(FORWARD_SPEED);
                    robot.rightDrive.setPower(FORWARD_SPEED);
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < .3)) {
                        telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }
                }
                telemetry.update();

                if (!rampUp)
                    // Keep stepping down until we hit the min value.
                    position -= INCREMENT;
                if (position <= MIN_POS) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servo to the new position and pause;
            robot.armServo.setPosition(position);
            sleep(CYCLE_MS);
            idle();

        }


        robot.leftDrive.setPower(FORWARD_SPEED);
        robot.rightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.leftDrive.setPower(FORWARD_SPEED);
        robot.rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();

        }
        robot.leftDrive.setPower(FORWARD_SPEED);
        robot.rightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
//    String format(OpenGLMatrix transformationMatrix) {
//        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
//    }
    }
}