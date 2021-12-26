package org.firstinspires.ftc.teamcode.robot.TFODOMH.TFMaths;

import java.io.IOException;

//this could've been a wrapper for GLMatrix, but I decided it would be fun to manually implement this

public class Matrix4f {

    private float[] m;

    public Matrix4f(){
        m = new float[] {1, 0, 0, 0,
                         0, 1, 0, 0,
                         0, 0, 1, 0,
                         0, 0, 0, 1};
    }

    public Matrix4f(float[] newMatrix){
        if (newMatrix.length == 16){
            m = newMatrix;
        } else {
            IOException e = new IOException("Matrix Length is not 9!");
            e.printStackTrace();
        }
    }

    public Vector4f matMul(Vector4f a){
        float nx = a.getX() * m[0] + a.getY() * m[1] + a.getZ() * m[2] + a.getW() * m[3];
        float ny = a.getX() * m[4] + a.getY() * m[5] + a.getZ() * m[6] + a.getW() * m[7];
        float nz = a.getX() * m[8] + a.getY() * m[9] + a.getZ() * m[10] + a.getW() * m[11];
        float nw = a.getX() * m[12] + a.getY() * m[12] + a.getZ() * m[13] + a.getW() * m[14];
        return new Vector4f(nx, ny, nz, nw);
    }

    public static Matrix4f matMul(Matrix4f a, Matrix4f b){
        float[][] cols = new float[][] {b.getCol(0), b.getCol(1), b.getCol(2), b.getCol(3)};
        float[][] rows = new float[][] {a.getRow(0), a.getRow(1), a.getRow(2), a.getRow(3)};
        float[] output = new float[16];

        for (int i = 0; i < 3; i++) {
            output[0 + i * 4] = cols[0][0] * rows[i][0] + cols[0][1] * rows[i][1] + cols[0][2] * rows[i][2] + cols[0][3] * rows[i][3];
            output[1 + i * 4] = cols[1][0] * rows[i][0] + cols[1][1] * rows[i][1] + cols[1][2] * rows[i][2] + cols[1][3] * rows[i][3];
            output[2 + i * 4] = cols[2][0] * rows[i][0] + cols[2][1] * rows[i][1] + cols[2][2] * rows[i][2] + cols[2][3] * rows[i][3];
            output[3 + i * 4] = cols[3][0] * rows[i][0] + cols[3][1] * rows[i][1] + cols[3][2] * rows[i][2] + cols[3][3] * rows[i][3];
        }

        return new Matrix4f(output);
    }

    public void add(Matrix4f b){
        for (int i = 0; i < 16; i++){
            this.m[i] += b.m[i];
        }
    }

    public void mul(float b){
        for (int i = 0; i < 16; i++){
            this.m[i] *= b;
        }
    }

    public static Matrix3f add(Matrix4f a, Matrix4f b){
        float[] newMat = new float[16];
        for (int i = 0; i < 9; i++){
            newMat[i] = a.m[i] + b.m[i];
        }

        return new Matrix3f(newMat);
    }

    public static Matrix3f mul(Matrix4f a, float b){
        float[] newMat = new float[16];
        for (int i = 0; i < 9; i++){
            newMat[i] = a.m[i] * b;
        }

        return new Matrix3f(newMat);
    }

    /**
     * [0,  1, 2, 3]
     * [4,  5, 6, 7]
     * [8,  9,10,11]
     * [12,13,14,15]
     * @return
     */
    public float det(){
        float a = TFMathExtension.quick3fArrDet(new float[] {m[5], m[6], m[7], m[9], m[10], m[11], m[13], m[14], m[15]}) * m[0];
        float b = TFMathExtension.quick3fArrDet(new float[] {m[4], m[6], m[7], m[8], m[10], m[11], m[12], m[14], m[15]}) * m[1];
        float c = TFMathExtension.quick3fArrDet(new float[] {m[4], m[5], m[7], m[8], m[9],  m[11], m[12], m[13], m[15]}) * m[2];
        float d = TFMathExtension.quick3fArrDet(new float[] {m[4], m[5], m[6], m[8], m[9],  m[10], m[12], m[13], m[14]}) * m[3];

        return a - b + c - d;
    }

    public void transpose(){
        float[] out = {m[0], m[4],  m[8], m[12],
                       m[1], m[5],  m[9], m[13],
                       m[2], m[6], m[10], m[14],
                       m[3], m[7], m[11], m[15]};
        m = out;
    }

    public void invert(){
        if (this.det() == 0){return;}
        float det_inv = 1 / this.det();

        float[] cofactor_mat = {this.getMinorDet(0), -this.getMinorDet(4), this.getMinorDet(8), -this.getMinorDet(12),
                                -this.getMinorDet(1), this.getMinorDet(5), -this.getMinorDet(9), this.getMinorDet(13),
                                this.getMinorDet(2), -this.getMinorDet(6), this.getMinorDet(10), -this.getMinorDet(14),
                                -this.getMinorDet(3), this.getMinorDet(7), -this.getMinorDet(11), this.getMinorDet(15)};

        for (int i = 0; i < cofactor_mat.length; i++){
            cofactor_mat[i] *= det_inv;
        }

        this.m = cofactor_mat;
    }

    public float[] getRow(int idx){
        return new float[] {m[idx * 4], m[idx * 4 + 1], m[idx * 4 + 2], m[idx * 4 + 3]};
    }

    public float[] getCol(int idx){
        return new float[] {m[idx], m[idx + 4], m[idx + 8], m[idx + 12]};
    }

    public float getMinorDet(int element_idx){
        int col = element_idx % 4, row = element_idx / 4, l = 0;
        float[] out3f = new float[9];

        for (int ex = 0; ex < 4; ex++){
            for (int ey = 0; ey < 4; ey++){
                if (ex != col || ey != row){
                    out3f[l] = m[ex + ey * 4];
                    l++;
                }
            }
        }

        return TFMathExtension.quick3fArrDet(out3f);
    }

    public float[] getAsFloatArray() {return this.m;}

    public Matrix4f getAsInversion(){
        if (this.det() == 0){return null;}
        float det_inv = 1 / this.det();

        float[] cofactor_mat = {this.getMinorDet(0), -this.getMinorDet(4), this.getMinorDet(8), -this.getMinorDet(12),
                -this.getMinorDet(1), this.getMinorDet(5), -this.getMinorDet(9), this.getMinorDet(13),
                this.getMinorDet(2), -this.getMinorDet(6), this.getMinorDet(10), -this.getMinorDet(14),
                -this.getMinorDet(3), this.getMinorDet(7), -this.getMinorDet(11), this.getMinorDet(15)};

        for (int i = 0; i < cofactor_mat.length; i++){
            cofactor_mat[i] *= det_inv;
        }

        return new Matrix4f(cofactor_mat);
    }

    @Override
    public String toString(){
       return ("\n" +  m[0] + " " +  m[1] + " " +  m[2] + " " +  m[3] +
               "\n" +  m[4] + " " +  m[5] + " " +  m[6] + " " +  m[7] +
               "\n" +  m[8] + " " +  m[9] + " " + m[10] + " " + m[11] +
               "\n" + m[12] + " " + m[13] + " " + m[14] + " " + m[15]);
    }
}