/**
  *Air traffic DeltaBot Control
  *Samuel Whitby | University of New South Wales
  */

int pathID = 0;
int[][][] myPaths = new int[10][2][2];//{line,x component,y component}
public PImage bg;
int bgON = 0;


void setup() {
  size(600, 405);
  bg = loadImage("background.jpg");
  background(bg);
} 

float midpoint (float x1, float x2){
  float m;
  m = (x1+x2)/2;
  return m;
}

void draw() {
  int i;
  float len;
  int midX, midY, y, x;
  int x1;
  int x2;
  int y1;
  int y2;
  int r, g, b;
  float m, c;
  stroke(200,30,30);
  strokeWeight(1);
  if(keyPressed) {
    if(key == 'c' || key == 'C') {
      bgON = 1;
    }
    else if(key == 'a' || key == 'A') {
      bgON = 0;
    }
  } 
  if( bgON == 1 ){
    background(bg);
  }
  for (i = 0; i < 10; i++){
    x1 = myPaths[i][0][0];
    x2 = myPaths[i][0][1];
    y1 = myPaths[i][1][0];
    y2 = myPaths[i][1][1];
    stroke(250,100+(i*12),5);
    line(x1, y1, x2, y2);
    len = dist(x1, y1, x2, y2);
    stroke(150-(i*5));
    if (x2-x1 != 0){
      m = (y2-y1)/(x2-x1);
    }
    else  m = (y2-y1)/(1);
    c = y1-m*x1;
    m = m * -1;
    midX = int(midpoint(x1, x2));
    midY = int(midpoint(y1, y2));
    c = midY-m*midX;
    x = int(midX - len/2);
    y = int(m*x+c);
    line(x-x1, y-y1, int(midX), int(midY));
  }
}
  
void mouseReleased() {
    myPaths[pathID][0][0] = int(470);
    myPaths[pathID][0][1] = int(random(width));
    myPaths[pathID][1][0] = int(300);
    myPaths[pathID][1][1] = int(random(height));
    pathID += 1;
    if (pathID > 9) pathID = 0;
  
}
