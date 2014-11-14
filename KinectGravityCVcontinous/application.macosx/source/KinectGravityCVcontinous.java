import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import hypermedia.video.*; 
import org.openkinect.*; 
import org.openkinect.processing.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class KinectGravityCVcontinous extends PApplet {

          
/**
 * Gravity System 
 * Graviy using open Kinect lib 
 */




float deg = 15; // Start at 15 degrees
boolean diagState=false;
boolean displayState=true;

//Adjustible parameters
int numBalls = 500;
float softening = 3.0f;
float vScale=0.10f;
float G=0.2f;
int simulationTime = 100;  //how long simulation runs
int drawingTime = 3000;  //time to draw
int tbuffer = 300;
float chance = 25.f; // percentage of time particles are drawn
OpenCV opencv;
PImage depth;
PImage icon_setup,icon_run;
int nB=0;
int gameStatus=0; // 0=paint mode, 1=gravity mode, 2 screensaver mode
float spring = 0.05f;
float gravity = 300.f;
float friction = .15f; //only used in moi
float cutoff =700;
float xscale = 1440/640.f;
float yscale = 1080/480.f;
float dwidth=1440;
int xoffset=270;
int currColor = 0;
int time=0;  //ticking
int baseTime=0;  //
int[] pBlobColor = new int[10];
int[] blobColor = new int[10];

// Kinect Obj
Kinect kinect;
// Size of kinect image
int kw = 640;
int kh = 480;
int threshold = 1500;

Ball[] balls = new Ball[numBalls];
int nColors=10;
int nColor=0;
int[] pColor =  new int[nColors];
PVector v;
Blob[] pBlobs, blobs;
int white = color(255,255,255,128);
int whiter = color(255,255,255,255);
//Fonts
PFont fontA;
PFont fontB;



public void setup() 
{
  size(1920, 1080);
  noCursor();
  kinect = new Kinect(this);
  kinect.start();
  kinect.enableDepth(true);
  // open video stream
  

  v=new PVector(0.f,0.f,0.f);
  //pBlobs=[];
  opencv = new OpenCV( this );
  opencv.allocate(640, 480 );
  depth = kinect.getDepthImage();
  opencv.copy(depth);
 
  pBlobs = opencv.blobs(20, 100000, 10, false );

  fontB = loadFont("AvenirNextLTPro-Bold-48.vlw");
  fontA = loadFont("AvenirNextLTPro-Bold-36.vlw");
  textFont(fontB);
  textAlign(CENTER);

  smooth();
  pColor[0]=color(255,255,55,155);
  pColor[1]=color(50,150,255,155);
  pColor[2]=color(255,128,0,155);
  pColor[3]=color(0,255,150,155);
  pColor[4]=color(0,255,0,155);
  pColor[5]=color(255,0,0,155);
  pColor[6]=color(0,0,255,155);
  pColor[7]=color(128,255,55,155);
  pColor[8]=color(128,50,255,155);
  pColor[9]=color(255,50,255,155);
  for (int i=0;i<nColors;i++) pBlobColor[i]=i;
  //icon_setup =loadImage("GS_setitup.png");
  //icon_run =loadImage("GS_runit.png");
  }

public void draw() 
{
  //println(frameRate);
  if (nB<10) background(50); else background(0);
  fill(255);
  if (diagState) {
    //textMode(SCREEN);
    textAlign(LEFT);
    if (gameStatus==0) text("Kinect FR: " + (int)kinect.getDepthFPS(),10,16);
    text("Processing FR: " + (int)frameRate,10,36);
    text("Number of Particles =: " + min(nB,numBalls),10,56);
    textAlign(CENTER);
  }
  checkTime();
  if (gameStatus==0) {
    paintDisplay();
  } else {
    simDisplay();
  }
  float vxin=0.f;
  float vyin=0.f;
  float distx=0.f;
  float disty=0.f;
  int ballColor;
  boolean matched;
  
  //Open CV Stuff

  if (gameStatus == 0 ) {
    depth = kinect.getDepthImage();
    opencv.copy(depth);
    opencv.threshold(137);    // set black & white threshold 
    if (displayState) {
      image(opencv.image(),20,100,130,120);
      image(depth,20,260,160,120);
    }
    float radius=0.f;
    // find blobs
    blobs = opencv.blobs( 100, 100000, 10, false , 500);
    // draw blob results
    for( int i=0; i<blobs.length; i++ ) {
      vxin=0.f;
      vyin=0.f;
      // Find Velocities
      matched=false;
      for( int k=0; k<pBlobs.length; k++ ) {
        distx=blobs[i].centroid.x-pBlobs[k].centroid.x;
        disty=blobs[i].centroid.y-pBlobs[k].centroid.y;
        float dist = sqrt(distx*distx+disty*disty);
        if (dist < 100.f) {
          matched=true;
          vxin=-1.0f*vScale*distx;
          vyin=1.0f*vScale*disty;
          blobColor[i]=pBlobColor[k];
        }
      }
        if (!matched) {
          currColor = (currColor +1) % nColors;
          blobColor[i] =currColor;
        } 
        if (blobs[i].area > 50) {
          fill(pColor[blobColor[i]]);
          stroke(128);
          beginShape();
          for( int j=0; j<blobs[i].points.length; j++ ) {
              vertex( width-xoffset-xscale*blobs[i].points[j].x, yscale*blobs[i].points[j].y );
          }
          endShape(CLOSE);
          radius=sqrt(blobs[i].area)/3.14159f;
          fill(255,255,255,100);
          noStroke();          
          ellipse(width-xoffset - xscale*blobs[i].centroid.x,yscale*blobs[i].centroid.y,radius,radius);
          if (random(100) < chance) {
            float theta = random(TWO_PI);
            float r=random(radius/2.f);
            float xoff = r*cos(theta);
            float yoff = r*sin(theta);
            balls[nB % numBalls] = new Ball(width +xoff-xoffset-PApplet.parseInt(xscale*blobs[i].centroid.x), yoff+PApplet.parseInt(height*blobs[i].centroid.y/(1.0f*kh)), vxin,vyin, radius, nB % numBalls,blobColor[i],  balls);
            nB++;
          }
        }
    }
    pBlobs=blobs;
    pBlobColor=blobColor;
    opencv.restore();
  }

  // Check Mouse to Spawn
  if (mousePressed == true && nB<numBalls) {
    float nY=random(10)+mouseY;
    vxin = friction*(mouseX-pmouseX);
    vyin = friction*(mouseY-pmouseY);
    if (nB>numBalls) {
      nB=0;
    }
    balls[nB] = new Ball(mouseX, nY, vxin,vyin, 3, nB, 0, balls);
    nB++;
  } 
  
  
// Display
    
    for (int i = 0; i < min(nB,numBalls); i++) {
      if (gameStatus==0) {
        balls[i].gravitate();
        balls[i].move();
      }
      balls[i].display();  
      balls[i].reset();
    }
  if (nB < 10){
    textFont(fontB);
    int ypos=frameCount-baseTime;
    fill(lerpColor(pColor[1],whiter,(ypos%(height+100))/height));
    text("GRAVITY SIMULATOR",width/2,ypos%(height+100));
    int ypos1=ypos-350;
    textFont(fontA);
    fill(lerpColor(pColor[3],whiter,ypos1%(height+100)/height));
    text("come close to the screen",width/2,ypos1%(height+100));
    int ypos2=ypos-500;
    fill(lerpColor(pColor[4],whiter,ypos2%(height+100)/height));
    text("and wave your hands to release particles",width/2,ypos2%(height+100));
  }
}

public void checkTime() {
  time = frameCount - baseTime;
  println(time+" "+simulationTime);
  if (gameStatus==0 && time >drawingTime/2 ) {
    if (nB < 10) {
      for (int i=0;i<100;i++) {
          balls[nB % numBalls] = new Ball(random(-width/2.f,+3*width/2), random(-height/2,3*height/2), 0,0, random(3,8), nB % numBalls,5,  balls);
          nB++;   
      }
    }
    if (time==drawingTime) {
      baseTime=frameCount;
      gameStatus=1;
      kinect.enableDepth(false);
      opencv.stop();
    }
//    if (time%1000==0) {
//      kinect.enableDepth(false);
//      opencv.stop();
//    }
//    if (time%1000==1) {
//      kinect.enableDepth(true);
//      opencv.allocate(640, 480 );
//    }
  } else if (gameStatus==1 && time==simulationTime) {
    gameStatus=0;
    kinect.enableDepth(true);
    opencv.allocate(640, 480 );
    nB=0;
    //System.gc();
  } 
}


class Ball {
  float x, y,vx,vy;
  float diameter;
  float ax = 0;
  float ay = 0;
  float mass = 1.f;
  int nC=0;
  int id;
  Ball[] cloud;
  
  Ball(float xin, float yin, float vxin, float vyin, float din, int idin,  int nColor, Ball[] oin) {
    x = xin;
    y = yin;
    vx = vxin;
    vy = vyin;
    diameter = din;
    mass=diameter*diameter/20.f;
    id = idin;
    cloud = oin;
    nC=nColor;
  } 
  
  public void gravitate() {
    for (int i = id + 1; i < min(nB,numBalls); i++) {
      float dx = cloud[i].x - x;
      float dy = cloud[i].y - y;
      float force=0;
      float a1=0.f;
      float a2=0.f;
      float distance = sqrt(dx*dx + dy*dy);
      if (distance >softening) {
        if (distance <cutoff) force = G*mass*cloud[i].mass/(distance*distance);
        a1=(force/mass)*(dx/distance);
        a2=(force/mass)*(dy/distance);
      }
      else {
        force = G*mass*cloud[i].mass/(softening*softening);
        a1=(force/mass)*(dx/softening);
        a2=(force/mass)*(dy/softening);
      }
      ax+=a1;
      ay+=a2;
      cloud[i].ax-=a1*mass/cloud[i].mass;
      cloud[i].ay-=a2*mass/cloud[i].mass;
    }   
  }
 
  public void reset(){
      ax=0;
      ay=0;
  }
  
  public void move() {
    vy += ay;
    vx += ax;
    x += vx;
    y += vy;
  }
  
  public void display() {
    if (gameStatus==0) {
      fill(pColor[nC]);
    } else {
      fill(lerpColor(pColor[nC],white,time/(1.0f*simulationTime)));
    }
    noStroke();
    ellipse(x, y, diameter, diameter);
    stroke(color(255,255,255,55));
    v.set(vx,vy,0.f);
    if (v.mag()>0.25f) {
      displayVector(v,x,y,10.f);
    }
  }
}

public void displayVector(PVector v, float x, float y, float scayl) {
  pushMatrix();
  float arrowsize = 4;
  // Translate to location to render vector
  translate(x,y);
  // Call vector heading function to get direction (note that pointing up is a heading of 0) and rotate
  rotate(v.heading2D());
  // Calculate length of vector & scale it to be bigger or smaller if necessary
  float len = v.mag()*scayl;
  // Draw three lines to make an arrow (draw pointing up since we've rotate to the proper direction)
  line(0,0,len,0);
  line(len,0,len-arrowsize,+arrowsize/2);
  line(len,0,len-arrowsize,-arrowsize/2);
  popMatrix();
} 

public void simDisplay() {
    //text("Gravity simulation running. You cannot add new particles now.", 512,25);
    //text("Simulation ends in ... "+ (int)(50 * (simulationTime-time)/simulationTime), 512,50);
    //image(icon_run, 5,760);
}

public void paintDisplay(){
    //text("Move close to the handrail and use your hands to place particles.", 512,25);
    if ((drawingTime-time)<tbuffer) text("Gravity simulation resets in ... "+ (int)(10 * (drawingTime-time)/tbuffer), width/2,height/2);
    //image(icon_setup, 5,760);
}



public void keyPressed() {

  if (key == CODED) {
    if (keyCode == UP) {
      deg++;
    } 
    else if (keyCode == DOWN) {
      deg--;
    }
    deg = constrain(deg,0,30);
    kinect.tilt(deg);
  }
  if (key=='d') {
    diagState=!diagState;
  }
  if (key=='s') {
    displayState=!displayState;
  }
  
}
public void stop() {
  kinect.quit();
  super.stop();
}

  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "--full-screen", "--bgcolor=#666666", "--stop-color=#cccccc", "KinectGravityCVcontinous" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
