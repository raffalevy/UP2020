package org.firstinspires.ftc.teamcode;

public class Stone {
    private int x;
    private int y;
    private boolean isSkystone;
    public int getX (){
        return x;
    }
    public int getY (){
        return y;
    }
    public void setX(int x1){
        x = x1;
    }
    public void setY(int y1){
        y = y1;
    }
    public boolean isSkystone(){
        return isSkystone;
    }
    public void setSkystone(boolean skystone) {
        isSkystone = skystone;
    }
    public Stone(int centerX, int centerY, boolean givenIsSkystone){
        x = centerX;
        y = centerY;
        isSkystone = givenIsSkystone;
    }
    public Stone(int centerX, int centerY) {
        x = centerX;
        y = centerY;
    }
    public Stone() {
    }
}
