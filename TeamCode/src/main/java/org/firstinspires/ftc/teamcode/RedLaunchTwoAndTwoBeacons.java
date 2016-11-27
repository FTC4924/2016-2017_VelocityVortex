package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/22/2016.
 */

@Autonomous(name = "RedLaunchTwoAndTwoBeacons")
public class RedLaunchTwoAndTwoBeacons extends LaunchTwoAndTwoBeaconsBase {

    public RedLaunchTwoAndTwoBeacons() {

        launchPositioningPath = new DrivePathSegment[]{

                new DrivePathSegment(12.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(1.0f),
        };

        beaconPath = new DrivePathSegment[]{

                new DrivePathSegment(315.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(35.0f, 1.0f, DrivePathSegment.LINEAR),
                new DrivePathSegment(270.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(20.0f, 0.5f, DrivePathSegment.LINEAR),
                new DrivePathSegment(270.0f, 0.2f, DrivePathSegment.TURN),
                new DrivePathSegment(3.0f, 0.1f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public boolean isRed() {

        return true;
    }
}