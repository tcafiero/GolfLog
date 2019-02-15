import processing.serial.*; //<>//

Table table;

class raw {
  float x;
  float y;
  float z;
};

raw acceleration = new raw();
raw gyroscope = new raw();
raw magnetic = new raw();

JSONObject json;

int W=1400; //My Laptop's screen width 
int H=700;  //My Laptop's screen height 
float Pitch; 
float Bank; 
float Azimuth; 
float heading; 
float ArtificialHoizonMagnificationFactor=0.7; 
float CompassMagnificationFactor=0.85; 
float SpanAngle=120; 
int NumberOfScaleMajorDivisions; 
int NumberOfScaleMinorDivisions; 
PVector v1, v2; 
float DECLINATION=3.52; // Declination (degrees) in Naples.


Serial port;
String my_port = "COM40";      //choose your port
float Phi;    //Dimensional axis
float Theta;
float Psi;

void setup() 
{ 
  //size(W, H); 
  //size(800, 600); 
  rectMode(CENTER); 
  size(1350, 690, P3D);
  smooth(); 
  strokeCap(SQUARE);//Optional 
  port = new Serial(this, my_port, 115200);
  port.bufferUntil('\n');
  table = new Table();
  //add a column header "Data" for the collected data
  table.addColumn("ts");
  table.addColumn("ax");
  table.addColumn("ay");
  table.addColumn("az");
  table.addColumn("gx");
  table.addColumn("gy");
  table.addColumn("gz");
  table.addColumn("mx");
  table.addColumn("my");
  table.addColumn("mz");
  //println(Serial.list()); //Shows your connected serial ports
}
void draw() 
{ 
  background(0); 
  translate(W/4, H/2.1);  
  MakeAnglesDependentOnMPU6050(); 
  Horizon(); 
  rotate(-Bank); 
  PitchScale(); 
  Axis(); 
  rotate(Bank); 
  Borders(); 
  Plane(); 
  ShowAngles(); 
  Compass(); 
  ShowAzimuth();
}
void serialEvent(Serial myPort) //Reading the datas by Processing.
{
  float roll, pitch, yaw, yaw2;
  int Timestamp;
  try {
    String myString = myPort.readStringUntil('\n');
    println(myString);
    JSONObject json = parseJSONObject(myString);
    Timestamp = json.getInt("ts");
    JSONArray accel = json.getJSONArray("a");
    JSONArray gyro = json.getJSONArray("g");
    JSONArray mag = json.getJSONArray("m");
    acceleration.x=accel.getFloat(0);
    acceleration.y=accel.getFloat(1);
    acceleration.z=accel.getFloat(2);
    gyroscope.x=gyro.getFloat(0);
    gyroscope.y=gyro.getFloat(1);
    gyroscope.z=gyro.getFloat(2);
    magnetic.x=mag.getFloat(0);
    magnetic.y=mag.getFloat(1);
    magnetic.z=mag.getFloat(2);
    //add a new row for each value
    TableRow newRow = table.addRow();
    //place the new row and value under the "Data" column
    newRow.setInt("ts", Timestamp);
    newRow.setFloat("ax", acceleration.x);
    newRow.setFloat("ay", acceleration.y);
    newRow.setFloat("az", acceleration.z);
    newRow.setFloat("gx", gyroscope.x);
    newRow.setFloat("gy", gyroscope.y);
    newRow.setFloat("gz", gyroscope.z);
    newRow.setFloat("mx", magnetic.x);
    newRow.setFloat("my", magnetic.y);
    newRow.setFloat("mz", magnetic.z);


    roll = atan2(acceleration.y, acceleration.z);
    if (acceleration.y * sin(roll) + acceleration.z * cos(roll) == 0)
      pitch = acceleration.x > 0 ? (PI / 2) : (-PI / 2);
    else
      pitch = (float)atan(-acceleration.x / (acceleration.y * sin(roll) + acceleration.z * cos(roll)));
    yaw = atan2((magnetic.z * sin(roll) - magnetic.y * cos(roll)), (magnetic.x * cos(pitch) + magnetic.y * sin(pitch) * sin(roll) + magnetic.z * sin(pitch) * cos(roll)));
    Azimuth=degrees(yaw);
    Pitch=degrees(pitch)*5;
    Bank=roll;
    //yaw = 180 * atan2(magnetic.x, magnetic.y)/PI;
    /*
    println("ax: "+accel.getInt(0)+" ay: "+accel.getInt(1)+" az: "+accel.getInt(2));
    println("gx: "+gyro.getInt(0)+" gy: "+gyro.getInt(1)+" gz: "+gyro.getInt(2));
    println("mx: "+mag.getInt(0)+" gy: "+mag.getInt(1)+" gz: "+mag.getInt(2));
    float   mx = mag.getInt(0);
    float   my = mag.getInt(1);
    float   mz = mag.getInt(2);
    float  norm = sqrt(mag.getInt(0) * mag.getInt(0) + mag.getInt(1) * mag.getInt(1) + mag.getInt(2) * mag.getInt(2));
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;
    */
    heading = yaw;
    heading -= DECLINATION * PI / 180;
    if (heading > PI) heading -= (2 * PI);
    else if (heading < -PI) heading += (2 * PI);
    else if (heading < 0) heading += 2 * PI;
    Phi = -roll; //radians(roll); 
    Theta = degrees(pitch); //radians(pitch); 
    //Psi = degrees(yaw);
    Psi = degrees(yaw);
    println("roll: "+degrees(roll)+" pitch: "+degrees(pitch)+" yaw: "+degrees(yaw)+" heading: "+degrees(heading));    
    /*
  float sensors[] = float(split(myString, ':'));  
     roll = sensors[0];
     pitch = sensors[1];
     yaw = sensors[2];
     //print(sensors[0]);
     Phi = roll*(-5); //radians(roll); 
     Theta = degrees(pitch)/2; //radians(pitch); 
     Psi = degrees(yaw); 
     //Azimuth=degrees(yaw);
     println("roll: " + roll + " pitch: " + pitch + " yaw: " + yaw + "\n"); //debug
     */
  }
  catch (Exception e) {
    println("error");
    myPort.write("\n");
    delay(500);
    myPort.write("\n");
    delay(500);
    myPort.write("\n");
    delay(500);
    myPort.write("Send\n");
  }
}

void mousePressed() {
  println("Closing sketch");
  saveTable(table, "datalog.csv");
  port.write("\n");
  delay(500);
  port.write("StopSend\n");
  delay(500);
  exit();
}


void MakeAnglesDependentOnMPU6050() 
{ 
  /*
  Bank =-Phi/5; 
   Pitch=Theta*10; 
   Azimuth=Psi;
   */
  //Bank =-Phi; 
  //Pitch=Theta*5; 
  //Azimuth=Psi;
}
void Horizon() 
{ 
  scale(ArtificialHoizonMagnificationFactor); 
  noStroke(); 
  fill(0, 180, 255); 
  rect(0, -100, 900, 1000); 
  fill(95, 55, 40); 
  rotate(-Bank); 
  rect(0, 400+Pitch, 900, 800); 
  rotate(Bank); 
  rotate(-PI-PI/6); 
  SpanAngle=120; 
  NumberOfScaleMajorDivisions=12; 
  NumberOfScaleMinorDivisions=24;  
  CircularScale(); 
  rotate(PI+PI/6); 
  rotate(-PI/6);  
  CircularScale(); 
  rotate(PI/6);
}

void ShowAzimuth() 
 { 
 fill(50); 
 noStroke(); 
 rect(20, 470, 440, 50); 
 int Azimuth1=round(Azimuth); 
 textAlign(CORNER); 
 textSize(35); 
 fill(255); 
 text("Azimuth:  "+Azimuth1+" Deg", 80, 477, 500, 60); 
 textSize(40);
 fill(25, 25, 150);
 text("M.Furkan Bahat", -350, 477, 500, 60);
 }

void Compass() 
{ 
  translate(2*W/3, 0); 
  scale(CompassMagnificationFactor); 
  noFill(); 
  stroke(100); 
  strokeWeight(80); 
  ellipse(0, 0, 750, 750); 
  strokeWeight(50); 
  stroke(50); 
  fill(0, 0, 40); 
  ellipse(0, 0, 610, 610); 
  for (int k=255; k>0; k=k-5) 
  { 
    noStroke(); 
    fill(0, 0, 255-k); 
    ellipse(0, 0, 2*k, 2*k);
  } 
  strokeWeight(20); 
  NumberOfScaleMajorDivisions=18; 
  NumberOfScaleMinorDivisions=36;  
  SpanAngle=180; 
  CircularScale(); 
  rotate(PI); 
  SpanAngle=180; 
  CircularScale(); 
  rotate(-PI); 
  fill(255); 
  textSize(60); 
  textAlign(CENTER); 
  text("W", -375, 0, 100, 80); 
  text("E", 370, 0, 100, 80); 
  text("N", 0, -365, 100, 80); 
  text("S", 0, 375, 100, 80); 
  textSize(30); 
  text("COMPASS", 0, -130, 500, 80); 
  rotate(PI/4); 
  textSize(40); 
  text("NW", -370, 0, 100, 50); 
  text("SE", 365, 0, 100, 50); 
  text("NE", 0, -355, 100, 50); 
  text("SW", 0, 365, 100, 50); 
  rotate(-PI/4); 
  CompassPointer();
}

void CompassPointer() 
{ 
  rotate(PI+radians(degrees(heading)));  
  stroke(0); 
  strokeWeight(4); 
  fill(100, 255, 100); 
  triangle(-20, -210, 20, -210, 0, 270); 
  triangle(-15, 210, 15, 210, 0, 270); 
  ellipse(0, 0, 45, 45);   
  fill(0, 0, 50); 
  noStroke(); 
  ellipse(0, 0, 10, 10); 
  triangle(-20, -213, 20, -213, 0, -190); 
  triangle(-15, -215, 15, -215, 0, -200); 
  rotate(-PI-radians(degrees(heading)));
}
void Plane() 
{ 
  fill(0); 
  strokeWeight(1); 
  stroke(0, 255, 0); 
  triangle(-20, 0, 20, 0, 0, 25); 
  rect(110, 0, 140, 20); 
  rect(-110, 0, 140, 20);
}
void CircularScale() 
{ 
  float GaugeWidth=800;  
  textSize(GaugeWidth/30); 
  float StrokeWidth=1; 
  float an; 
  float DivxPhasorCloser; 
  float DivxPhasorDistal; 
  float DivyPhasorCloser; 
  float DivyPhasorDistal; 
  strokeWeight(2*StrokeWidth); 
  stroke(255);
  float DivCloserPhasorLenght=GaugeWidth/2-GaugeWidth/9-StrokeWidth; 
  float DivDistalPhasorLenght=GaugeWidth/2-GaugeWidth/7.5-StrokeWidth;
  for (int Division=0; Division<NumberOfScaleMinorDivisions+1; Division++) 
  { 
    an=SpanAngle/2+Division*SpanAngle/NumberOfScaleMinorDivisions;  
    DivxPhasorCloser=DivCloserPhasorLenght*cos(radians(an)); 
    DivxPhasorDistal=DivDistalPhasorLenght*cos(radians(an)); 
    DivyPhasorCloser=DivCloserPhasorLenght*sin(radians(an)); 
    DivyPhasorDistal=DivDistalPhasorLenght*sin(radians(an));   
    line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal);
  }
  DivCloserPhasorLenght=GaugeWidth/2-GaugeWidth/10-StrokeWidth; 
  DivDistalPhasorLenght=GaugeWidth/2-GaugeWidth/7.4-StrokeWidth;
  for (int Division=0; Division<NumberOfScaleMajorDivisions+1; Division++) 
  { 
    an=SpanAngle/2+Division*SpanAngle/NumberOfScaleMajorDivisions;  
    DivxPhasorCloser=DivCloserPhasorLenght*cos(radians(an)); 
    DivxPhasorDistal=DivDistalPhasorLenght*cos(radians(an)); 
    DivyPhasorCloser=DivCloserPhasorLenght*sin(radians(an)); 
    DivyPhasorDistal=DivDistalPhasorLenght*sin(radians(an)); 
    if (Division==NumberOfScaleMajorDivisions/2|Division==0|Division==NumberOfScaleMajorDivisions) 
    { 
      strokeWeight(15); 
      stroke(0); 
      line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal); 
      strokeWeight(8); 
      stroke(100, 255, 100); 
      line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal);
    } else 
    { 
      strokeWeight(3); 
      stroke(255); 
      line(DivxPhasorCloser, DivyPhasorCloser, DivxPhasorDistal, DivyPhasorDistal);
    }
  }
}
void Axis() 
{ 
  stroke(255, 0, 0); 
  strokeWeight(3); 
  line(-115, 0, 115, 0); 
  line(0, 280, 0, -280); 
  fill(100, 255, 100); 
  stroke(0); 
  triangle(0, -285, -10, -255, 10, -255); 
  triangle(0, 285, -10, 255, 10, 255);
}
void ShowAngles() 
{ 
  textSize(30); 
  fill(50); 
  noStroke(); 
  rect(-150, 400, 280, 40); 
  rect(150, 400, 280, 40); 
  fill(255); 
  //Pitch=Pitch; 
  int Pitch1=round(Pitch)/5;  
  text("Pitch:  "+Pitch1+" Deg", -20, 411, 500, 60); 
  text("Bank:  "+Bank*100+" Deg", 280, 411, 500, 60);
}
void Borders() 
{ 
  noFill(); 
  stroke(0); 
  strokeWeight(400); 
  rect(0, 0, 1100, 1100); 
  strokeWeight(200); 
  ellipse(0, 0, 1000, 1000); 
  fill(0); 
  noStroke(); 
  rect(4*W/5, 0, W, 2*H); 
  rect(-4*W/5, 0, W, 2*H);
}
void PitchScale() 
{  
  stroke(255); 
  fill(255); 
  strokeWeight(3); 
  textSize(24); 
  textAlign(CENTER); 
  for (int i=-4; i<5; i++) 
  {  
    if ((i==0)==false) 
    { 
      line(110, 50*i, -110, 50*i);
    }  
    text(""+i*10, 140, 50*i, 100, 30); 
    text(""+i*10, -140, 50*i, 100, 30);
  } 
  textAlign(CORNER); 
  strokeWeight(2); 
  for (int i=-9; i<10; i++) 
  {  
    if ((i==0)==false) 
    {    
      line(25, 25*i, -25, 25*i);
    }
  }
}
