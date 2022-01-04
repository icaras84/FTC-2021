package org.firstinspires.ftc.teamcode.robot.TFODOMH.ODMain;

import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Matrix4f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Plane3f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Transform4f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Vector2f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Vector3f;

public class FrustumInterpolator {

    private Transform4f worldTransform = null; //describe the local transform of the camera itself from the robot epicenter
    private Matrix4f cameraMatrix = null; //camera matrix will be used to modify frustum coordinates in local space

    private double hFOV, vFOV; //horizontal and vertical fov, important for calculating the frustum later
    private Vector3f fplane_right = new Vector3f(), fplane_bottom = new Vector3f(), fplane_center = new Vector3f();
    private Plane3f cardinalAxisPlane = Plane3f.X_PLANE;

    public FrustumInterpolator(double horizontal_fov, double vertical_fov){
        this.hFOV = horizontal_fov;
        this.vFOV = vertical_fov;

        this.worldTransform = new Transform4f();
        this.cameraMatrix = new Matrix4f();

        this.setupFrustum();
    }

    private void setupFrustum(){
        double angV = Math.toRadians(vFOV) / 2;
        double angH = Math.toRadians(hFOV) / 2;

        Vector2f fpbot = new Vector2f((float) -Math.sin(angV), (float) Math.cos(angV));
        Vector2f fpright = new Vector2f((float) Math.sin(angH), (float) Math.cos(angH));

        //coords are normalized
        fplane_bottom = new Vector3f(0, fpbot.getX(), fpbot.getY());
        fplane_right = new Vector3f(fpright.getX(), 0, fpright.getY());
        fplane_center = new Vector3f(0, 0, 1);

    }

}
