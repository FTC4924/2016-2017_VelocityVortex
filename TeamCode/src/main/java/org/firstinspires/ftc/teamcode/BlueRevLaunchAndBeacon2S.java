package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 4/7/2017.
 */

@Autonomous(name = "BlueRevLaunchAndBeacon2S", group = "Revolution")
public class BlueRevLaunchAndBeacon2S extends RevLaunchAndBeacon2SBase {

    public BlueRevLaunchAndBeacon2S() {

        launchPositioningPath = new DrivePathSegment[] {

                new DrivePathSegment(8.5f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(17.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(0.5f),
        };

        beaconPath = new DrivePathSegment[] {

                new DrivePathSegment(45.0f, 0.8f, DrivePathSegment.TURN),
                new DrivePathSegment(11.0f, 0.8f, DrivePathSegment.LINEAR),
                new DrivePathSegment(85.0f, 0.8f, DrivePathSegment.TURN),
                new DrivePathSegment(14.0f, 0.4f, DrivePathSegment.LINEAR),
                new DrivePathSegment(3.0f, 0.1f, DrivePathSegment.LINEAR),
        };

        leaveBeaconPath = new DrivePathSegment[] {

                new DrivePathSegment(-0.05f, 0.3f, DrivePathSegment.LINEAR),

        };
    }

    @Override
    public boolean isRed() {

        return false;
    }

    public float throwingPower() {

        return 0.9f;
    }
}

