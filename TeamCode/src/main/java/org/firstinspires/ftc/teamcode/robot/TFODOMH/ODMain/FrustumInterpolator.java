package org.firstinspires.ftc.teamcode.robot.TFODOMH.ODMain;

import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Matrix4f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Matrix4fBuilder;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Plane3f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.TFMathExtension;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Transform4f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Vector2f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Vector3f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Vector4f;

public class FrustumInterpolator {

    private Matrix4f imgToLocal = null; //camera matrix will be used to modify frustum coordinates in local space
    private Matrix4f camRot = null;

    private double hFOV, vFOV; //horizontal and vertical fov, important for calculating the frustum later
    private Vector4f fplane_right = new Vector4f(), fplane_bottom = new Vector4f(), fplane_center = new Vector4f();
    private Vector3f camPos = new Vector3f();
    private Plane3f cardinalAxisPlane = Plane3f.X_PLANE;

    //presets for listed cameras
    public static FrustumInterpolator Logitech_C270 = new FrustumInterpolator(TFMathExtension.convertDFov(55, 16, 9));

    public FrustumInterpolator(double horizontal_fov, double vertical_fov, Matrix4f cam_rot, Vector3f cam_pos){
        this.hFOV = horizontal_fov;
        this.vFOV = vertical_fov;

        this.imgToLocal = new Matrix4f();
        this.camRot = cam_rot;
        this.camPos = cam_pos;

        this.setupFrustum();
    }

    public FrustumInterpolator(double[] fov, Matrix4f cam_rot, Vector3f cam_pos){
        this.hFOV = fov[0];
        this.vFOV = fov[1];

        this.imgToLocal = new Matrix4f();
        this.camRot = cam_rot;
        this.camPos = cam_pos;

        this.setupFrustum();
    }

    public FrustumInterpolator(double[] fov){
        this.hFOV = fov[0];
        this.vFOV = fov[1];

        this.imgToLocal = new Matrix4f();
        this.camRot = new Matrix4f(); //cam_rot;
        this.camPos = new Vector3f(); //cam_pos;

        this.setupFrustum();
    }

    private void setupFrustum(){
        double angV = Math.toRadians(vFOV) / 2;
        double angH = Math.toRadians(hFOV) / 2;

        float fpDistance = 100;

        Vector3f temp = this.camPos;

        fplane_right = camRot.matMul(new Vector4f((float) Math.tan(1/2 * hFOV) * fpDistance, 0, 0, 1));
        fplane_bottom = camRot.matMul(new Vector4f(0, (float) -Math.tan(1/2 * vFOV) * fpDistance, 0, 1));
        fplane_center = camRot.matMul(new Vector4f(0, 0, 1, 1));

        imgToLocal = new Matrix4f(new float[]
                {fplane_right.getX(), fplane_bottom.getX(), fplane_center.getX(), camPos.getX(),
                 fplane_right.getY(), fplane_bottom.getY(), fplane_center.getY(), camPos.getY(),
                 fplane_right.getZ(), fplane_bottom.getZ(), fplane_center.getZ(), camPos.getZ(),
                                   0,                    0,                    0,             1}
                );
    }

    public Vector3f convertIMGCoord(Vector4f bb_pos){
        Vector3f output = this.cardinalAxisPlane.getVector3fInt(camPos, this.imgToLocal.matMul(bb_pos).getAsVec3f());
        return output;
    }

    /**
     * SETTERS AND GETTERS
     */

    public void setCamRot(Matrix4f newRotation){
        this.camRot = newRotation;
        this.setupFrustum();
    }

    public void setCamPos(Vector3f newPosition){
        this.camPos = newPosition;
        this.setupFrustum();
    }

}
