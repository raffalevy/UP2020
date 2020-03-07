/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "TensorFlowAuto", group = "Auto")
public class TensorFlowAutoBlue extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AcL2Ktb/////AAABmZ99Ni45mUanrZmv9MbOv9Yli7vaEQKEuwyAMPvPtbSLY6yqXIAzu4eyywbksWjQF0PE/lxiUfNysG7dUlB0gIp69YWWFP2p6MG7wdJSBuDWg488yJ8eeMbrEdkbKOD4XwhMJxgHkQqaMynd8sM01ijFP65RwwYfnnpn5vXYpQqSpMR3vv+DDgotyx1J9syA4ivw2CgFrA1Amq5HD19TbOXgOcnWv/KkauafHuWZprmUjJuv5pyYcnvsJ0NXqidLbtj1C1ftDSpm88ChzXOYKBjaG1sFDdZPZ8wUgO43/IYtisS2DnMT1lGDYEan7WpcUYEOECE5h0GcItN8vVqh7gNeH8/GVWn+zFl0QVjpRigE";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();

    private MecanumRobot rb = new MecanumRobot();

    private MecanumOdometry odometry = new MecanumOdometry();

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        Stone stone1 = new Stone(0,35);
        Stone stone2 = new Stone(8,35);
        Stone stone3 = new Stone(16,35);
        Stone stone4 = new Stone(24,35);
        Stone stone5 = new Stone(32,35);
        Stone stone6 = new Stone(40,35);

        List<Stone> allStones = new ArrayList<>();
        allStones.add(stone1);
        allStones.add(stone2);
        allStones.add(stone3);
        allStones.add(stone4);
        allStones.add(stone5);
        allStones.add(stone6);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        odometry.start(rb.frMotor.getCurrentPosition(),rb.flMotor.getCurrentPosition(),rb.blMotor.getCurrentPosition());
        waitForStart();

        while (opModeIsActive()) {
            rb.driveToY(12,.7,odometry,runtime,10);
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions.get(0).getLabel().equals("Skystone")){
                    stone1.setSkystone(true);
                } else {
                    rb.strafeToX(4.,.7,odometry,runtime,10);
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions.get(0).getLabel().equals("Skystone")) {
                        stone2.setSkystone(true);
                    }
                        else {
                            stone3.setSkystone(true);
                        }
                }
            }
                if (stone1.isSkystone()){
                    rb.strafeToX(stone1.getX() ,.7, odometry, runtime, 10);
                    rb.driveToY(stone1.getY() ,.7,odometry, runtime, 10);
                    rb.grab();
                    rb.driveToY(stone1.getY()-12,.7,odometry,runtime,10);
                    rb.goToTheta(Math.PI / 2, .7, odometry, runtime, 10);
                    rb.driveToX(50,.7, odometry, runtime,10);
                    rb.release();
                    rb.goToTheta(3 * Math.PI / 2, .7, odometry, runtime, 10);
                    rb.strafeToY(12,.7 ,odometry, runtime, 10);
                    rb.driveToX(stone4.getX(), .7, odometry, runtime, 10);
                    rb.goToTheta(0,.7,odometry,runtime,10);
                    rb.driveToY(stone4.getY(),.7,odometry,runtime,10);
                    rb.grab();
                    rb.driveToY(stone4.getY()-12,.7,odometry,runtime,10);
                    rb.goToTheta(Math.PI / 2, .7, odometry, runtime, 10);
                    rb.driveToX(50,.7, odometry, runtime,10);
                    rb.release();
                    rb.strafeToY(0,.7,odometry,runtime,10);
                    rb.driveToX(70,.7, odometry, runtime, 10);
                    rb.goToTheta(Math.PI,.7, odometry, runtime, 10);
                    rb.driveToY(48,.7,odometry,runtime, 10);
                    rb.hooksDown();
                    rb.driveToY(0, .7, odometry, runtime, 10);
                    rb.hooksUp();
                    rb.strafeToX(30, .7, odometry, runtime, 10);
                    rb.driveToY(30, .7, odometry, runtime, 10);
                    rb.strafeToX(24, .7, odometry, runtime, 10);
                    stop();
                } else if (stone2.isSkystone()){
                    rb.strafeToX(stone2.getX(),.7, odometry, runtime, 10);
                    rb.driveToY(stone2.getY() ,.7,odometry, runtime, 10);
                    rb.grab();
                    rb.driveToY(stone2.getY()-12,.7,odometry,runtime,10);
                    rb.goToTheta(Math.PI / 2, .7, odometry, runtime, 10);
                    rb.driveToX(50,.7, odometry, runtime,10);
                    rb.release();
                    rb.goToTheta(3 * Math.PI / 2, .7, odometry, runtime, 10);
                    rb.strafeToY(12,.7 ,odometry, runtime, 10);
                    rb.driveToX(stone5.getX(), .7, odometry, runtime, 10);
                    rb.goToTheta(0,.7,odometry,runtime,10);
                    rb.driveToY(stone5.getY(),.7,odometry,runtime,10);
                    rb.grab();
                    rb.driveToY(stone5.getY()-12,.7,odometry,runtime,10);
                    rb.goToTheta(Math.PI / 2, .7, odometry, runtime, 10);
                    rb.driveToX(50,.7, odometry, runtime,10);
                    rb.release();
                    rb.strafeToY(0,.7,odometry,runtime,10);
                    rb.driveToX(70,.7, odometry, runtime, 10);
                    rb.goToTheta(Math.PI,.7, odometry, runtime, 10);
                    rb.driveToY(48,.7,odometry,runtime, 10);
                    rb.hooksDown();
                    rb.driveToY(0, .7, odometry, runtime, 10);
                    rb.hooksUp();
                    rb.strafeToX(30, .7, odometry, runtime, 10);
                    rb.driveToY(30, .7, odometry, runtime, 10);
                    rb.strafeToX(24, .7, odometry, runtime, 10);
                } else if (stone3.isSkystone()){
                    rb.strafeToX(stone3.getX(),.7, odometry, runtime, 10);
                    rb.driveToY(stone3.getY() ,.7,odometry, runtime, 10);
                    rb.grab();
                    rb.driveToY(stone3.getY()-12,.7,odometry,runtime,10);
                    rb.goToTheta(Math.PI / 2, .7, odometry, runtime, 10);
                    rb.driveToX(50,.7, odometry, runtime,10);
                    rb.release();
                    rb.goToTheta(3 * Math.PI / 2, .7, odometry, runtime, 10);
                    rb.strafeToY(12,.7 ,odometry, runtime, 10);
                    rb.driveToX(stone6.getX(), .7, odometry, runtime, 10);
                    rb.goToTheta(0,.7,odometry,runtime,10);
                    rb.driveToY(stone6.getY(),.7,odometry,runtime,10);
                    rb.grab();
                    rb.driveToY(stone6.getY()-12,.7,odometry,runtime,10);
                    rb.goToTheta(Math.PI / 2, .7, odometry, runtime, 10);
                    rb.driveToX(50,.7, odometry, runtime,10);
                    rb.release();
                    rb.strafeToY(0,.7,odometry,runtime,10);
                    rb.driveToX(70,.7, odometry, runtime, 10);
                    rb.goToTheta(Math.PI,.7, odometry, runtime, 10);
                    rb.driveToY(48,.7,odometry,runtime, 10);
                    rb.hooksDown();
                    rb.driveToY(0, .7, odometry, runtime, 10);
                    rb.hooksUp();
                    rb.strafeToX(30, .7, odometry, runtime, 10);
                    rb.driveToY(30, .7, odometry, runtime, 10);
                    rb.strafeToX(24, .7, odometry, runtime, 10);
                }
            }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minimumConfidence = 0.7;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    private boolean isSkystone(Recognition recognition) {
        if (recognition.getLabel().equals("Skystone")) {
            return true;
        } else {
            return false;
        }
    }
}
