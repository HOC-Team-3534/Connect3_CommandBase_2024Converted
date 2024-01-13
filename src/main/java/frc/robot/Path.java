package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;

public enum Path {
    BumpSide_PickUp_PlaceWhileMove_BalanceForward("Bump Side PickUp PlaceWhileMove BalanceForward"),
    BumpSide_PickUp_Third("Bump Side PickUp Third"),
    BumpSide_PickUp_PlaceSecond("Bump Side Pickup PlaceSecond"),
    BumpSide_PickUp_Balance("Bump Side PickUp Balance"),
    BumpSide_DriveForward("Bump Side Drive Forward"),
    LoadingZone_PickUp_PlaceWhileMove_BalanceForward("Loading Zone PickUp PlaceWhileMove BalanceForward"),
    LoadingZone_PickUp_Third("Loading Zone PickUp Third"),
    LoadingZone_PickUp_PlaceSecond("Loading Zone PickUp PlaceSecond"),
    LoadingZone_PickUp_Balance("Loading Zone PickUp Balance"),
    LoadingZone_DriveForward("Loading Zone Drive Forward");

    String pathName;
    PathPlannerPath path;

    Path() {
        this.pathName = this.name();
    }

    Path(String pathName) {
        this.pathName = pathName;
    }

    public void loadPath() {
        path = PathPlannerPath.fromPathFile(pathName);
    }

    public PathPlannerPath getPath() {
        return path;
    }
}