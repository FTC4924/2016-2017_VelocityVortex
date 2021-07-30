package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by 4924_Users on 4/1/2017.
 */

@Autonomous(name = "RedRevLaunchAndBeacon2S", group = "Revolution")
public class RedRevLaunchAndBeacon2S extends RevLaunchAndBeacon2SBase {

    public RedRevLaunchAndBeacon2S() {

        launchPositioningPath = new DrivePathSegment[] {

                new DrivePathSegment(5.5f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(350.0f, 0.8f, DrivePathSegment.TURN),
                new DrivePathSegment(5.0f, 0.8f, DrivePathSegment.TURN), //This has a reason behind it
                new DrivePathSegment(0.25f),
        };

        beaconPath = new DrivePathSegment[] {

                new DrivePathSegment(0.0f, 0.8f, DrivePathSegment.TURN),
                new DrivePathSegment(2.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(322.0f, 0.8f, DrivePathSegment.TURN),
                new DrivePathSegment(11.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(286.0f, 0.8f, DrivePathSegment.TURN),
                new DrivePathSegment(14.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(3.0f, 0.1f, DrivePathSegment.LINEAR),
                new DrivePathSegment(-3.0f, 0.1f, DrivePathSegment.LINEAR),
        };

        leaveBeaconPath = new DrivePathSegment[] {

                new DrivePathSegment(-0.05f, 0.3f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {

        return true;
    }

    public float throwingPower() {

        return 0.9f;
    }
}