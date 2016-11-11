/*
 * Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 */

package org.firstinspires.ftc.teamcode.VisionSamples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Vision Op Mode designed ONLY for testing applications, such as the Camera Test Activity
 * This OpMode essentially unifies testing applications and the robot controller
 */
@Autonomous(name="Vision Test", group="Robowiz")
@Disabled
public abstract class TestableVisionOpMode extends VisionOpMode {

    /**
     * Creates the Testable OpMode.
     */
    public TestableVisionOpMode() {
        super(true); //disable OpenCV core functions
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}