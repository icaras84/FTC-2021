package org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths;

public class TFMathExtension {

    public static float lerp(float a, float b, float t){
        return a + (b - a) * t;
    }

    public static Vector3f lerp(Vector3f a, Vector3f b, float t){
        float nX = a.getX() + (b.getX() - a.getX()) * t;
        float nY = a.getY() + (b.getY() - a.getY()) * t;
        float nZ = a.getZ() + (b.getZ() - a.getZ()) * t;

        return new Vector3f(nX, nY, nZ);
    }
}
