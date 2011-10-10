/**
  *Air traffic DeltaBot Control
  *Samuel Whitby | University of New South Wales
  */

int pathID = 0;
int[][][] myPaths = new int[10][2][2];//{line,x component,y component}


void setup() {
  size(300, 300);
} 

float midpoint (float x1, float x2){
  float m;
  m = (x1+x2)/2;
  return m;
}

void draw() {
  int i;
  //float h, len;
  //int midX, midY;
  int x1;
  int x2;
  int y1;
  int y2;
  int r, g, b;
  stroke(200,30,30);
  strokeWeight(2);
  background(0);
  if(mousePressed) {
    myPaths[pathID][0][0] = int(random(width));
    myPaths[pathID][0][1] = int(random(width));
    myPaths[pathID][1][0] = int(random(height));
    myPaths[pathID][1][1] = int(random(height));
    pathID += 1;
    if (pathID > 9) pathID = 0;
  }
  for (i = 0; i < 10; i++){
    x1 = myPaths[i][0][0];
    x2 = myPaths[i][0][1];
    y1 = myPaths[i][1][0];
    y2 = myPaths[i][1][1];
    stroke(200,30,30);
    line(x1, y1, x2, y2);
    //stroke(30,30,200);
    //noFill();
    //midX = int(midpoint(x1, x2));
    //midY = int(midpoint(y1, y2));
    //len = dist(x1, y1, x2, y2)/2;
    //h = int(random(5,25));
    //arc(midX, midY, len, h, PI/2,TWO_PI);
  }
}
