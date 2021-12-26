package org.firstinspires.ftc.teamcode.robot.TFODOMH.ODMain;

import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Matrix4f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Plane3f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Transform4f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Vector3f;

public class CamAttributes {

    private Transform4f worldTransform = null; //describe the local transform of the camera itself from the robot epicenter
    private Matrix4f cameraMatrix = null; //camera matrix will be used to modify frustum coordinates in local space
    private Matrix4f projectionMatrix = null; //projection matrix, will be inverted later

    private float hFOV, vFOV; //horizontal and vertical fov, important for calculating the frustum later
    private Vector3f fplane_right = new Vector3f(), fplane_bottom = new Vector3f(), fppoint = new Vector3f();
    private Plane3f cardinalAxisPlane = Plane3f.X_PLANE;

    public CamAttributes(){
        this.worldTransform = new Transform4f();
        this.cameraMatrix = new Matrix4f();
        this.projectionMatrix = new Matrix4f();

        this.setupFrustum();
    }

    private void setupFrustum(){

    }

}
