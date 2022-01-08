package org.firstinspires.ftc.teamcode.robot.TFODOMH.ODMain;

import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Matrix4f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Plane3f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.TFMathExtension;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Vector2f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Vector3f;
import org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths.Vector4f;

/**
 * This class simulates the viewable space of a camera
 */
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

    /**
     * Usually a method called in the constructor and setters to update the matrices used
     * <br> However, this method is available because this class doesn't work with MOMM
     */
    public void setupFrustum(){
        double angV = Math.toRadians(vFOV) / 2;
        double angH = Math.toRadians(hFOV) / 2;

        float fpDistance = 100;

        Vector3f temp = this.camPos;

        fplane_right = camRot.matMul(new Vector4f((float) Math.tan(0.5 * Math.toRadians(hFOV)) * fpDistance, 0, 0, 1));
        fplane_bottom = camRot.matMul(new Vector4f(0, (float) -Math.tan(0.5 * Math.toRadians(vFOV)) * fpDistance, 0, 1));
        fplane_center = camRot.matMul(new Vector4f(0, 0, fpDistance, 1));

        imgToLocal = new Matrix4f(new float[]
                {fplane_right.getX(), fplane_bottom.getX(), fplane_center.getX(), camPos.getX(),
                 fplane_right.getY(), fplane_bottom.getY(), fplane_center.getY(), camPos.getY(),
                 fplane_right.getZ(), fplane_bottom.getZ(), fplane_center.getZ(), camPos.getZ(),
                                   0,                    0,                    0,             1}
                );
    }

    /**
     * This converts the given Bounding Box coordinate passed with a 2d vector into a 3d coordinate on the XZ plane
     * @param bb_pos
     * @return XZ & bb_pos intersection
     */
    public Vector3f convertIMGCoord(Vector2f bb_pos){
        Vector3f output = this.cardinalAxisPlane.getVector3fInt(camPos, this.imgToLocal.matMul(new Vector4f(bb_pos.getX(), bb_pos.getY(), 1, 1)).getAsVec3f());
        return output;
    }

    /**
     * This converts the given Bounding Box coordinate passed with a 2d vector into a 3d coordinate on the XZ plane (4D input version)
     * @param bb_pos
     * @return XZ & bb_pos intersection
     */
    public Vector3f convertIMGCoord(Vector4f bb_pos){
        Vector3f output = this.cardinalAxisPlane.getVector3fInt(camPos, this.imgToLocal.matMul(bb_pos).getAsVec3f());
        return output;
    }

    /**
     * GETTERS AND SETTERS
     */

    public void setCamRot(Matrix4f newRotation){
        this.camRot = newRotation;
        this.setupFrustum();
    }

    public void setCamPos(Vector3f newPosition){
        this.camPos = newPosition;
        this.setupFrustum();
    }

    public Matrix4f getImgToLocal() {
        return imgToLocal;
    }

    public Matrix4f getCamRot() {
        return camRot;
    }

    public double gethFOV() {
        return hFOV;
    }

    public double getvFOV() {
        return vFOV;
    }

    public Vector4f getFplane_right() {
        return fplane_right;
    }

    public Vector4f getFplane_bottom() {
        return fplane_bottom;
    }

    public Vector4f getFplane_center() {
        return fplane_center;
    }

    public Vector3f getCamPos() {
        return camPos;
    }

    public Plane3f getCardinalAxisPlane() {
        return cardinalAxisPlane;
    }
}