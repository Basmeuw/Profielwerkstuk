public class Vector {

  float x, y;

  public Vector(){
    this.x = 0;
    this.y = 0;
  }
  
  public Vector(float x, float y) {
    this.x = x;
    this.y = y;
  }

  public void set(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public Vector copy(){
    return new Vector(this.x, this.y);
  }
  
  public void randomVector(float min, float max){
    this.x = random(min, max); 
    this.y = random(min, max);
  }
  
  public void add(Vector other) {
    this.x += other.x;
    this.y += other.y;
  }
  
  public void add(float x) {
    this.x += x;
    this.y += x;
  }
  
  public void add(float x, float y) {
    this.x += x;
    this.y += y;
  }
  
  public void sub(Vector other) {
    this.x -= other.x;
    this.y -= other.y;
  }
  
  public void sub(float x) {
    this.x -= x;
    this.y -= x;
  }
  
  public void sub(float x, float y) {
    this.x -= x;
    this.y -= y;
  }
  
  public void mult(Vector other) {
    this.x *= other.x;
    this.y *= other.y;
  }
  
  public void mult(float x) {
    this.x *= x;
    this.y *= x;
  }
  
  public void mult(float x, float y) {
    this.x *= x;
    this.y *= y;
  }
  
  public void div(Vector other) {
    this.x /= other.x;
    this.y /= other.y;
  }
  
  public void div(float x) {
    this.x /= x;
    this.y /= x;
  }
  
  public void div(float x, float y) {
    this.x /= x;
    this.y /= y;
  }
  
  public float mag(){
    return (float) sqrt(this.x*this.x + this.y*this.y);
  }
  
  public float magSqr(){
     return (this.x*this.x + this.y*this.y);
  }
  
  public void normalize(){
    this.div(this.mag());
  }
  
  public float dot(Vector v){
    return this.x * v.x + this.y * v.y;
  }
  
  public void constrain(float rx1, float rx2, float ry1, float ry2){
    if(this.x < rx1) this.x = rx1;
    if(this.x > rx2) this.x = rx2;
    if(this.y < ry1) this.y = ry1;
    if(this.y > ry2) this.y = ry2;
  }
  
  public Vector neg(){
    return new Vector(-this.x, -this.y);
  }
  
  
  public void display(){
    line(0, 0, this.x, this.y);
  }
  
  public void display(float multiplier){
    line(0, 0, this.x * multiplier, this.y * multiplier);
  }
 
  // Methods for printing
  public void vprint() {
    print(this.x + ", " + this.y);
  }

  public void vprintln() {
    println(this.x + ", " + this.y);
  }
}

// Methods that need to be called without having an instance of a Vector object
public Vector add(Vector v1, Vector v2) {
    return new Vector(v1.x + v2.x, v1.y + v2.y);
}

public Vector sub(Vector v1, Vector v2) {
    return new Vector(v1.x - v2.x, v1.y - v2.y);
}

public Vector mult(Vector v1, Vector v2) {
    return new Vector(v1.x * v2.x, v1.y * v2.y);
}

public Vector mult(Vector v, float val) {
    return new Vector(v.x * val, v.y * val);
}

public Vector div(Vector v1, Vector v2) {
    return new Vector(v1.x / v2.x, v1.y / v2.y);
}

public Vector div(Vector v, float val) {
    return new Vector(v.x / val, v.y / val);
}

public Vector normalize(Vector v){
  return (div(v, v.mag()));
}

public float dot(Vector v1, Vector v2){
  return (v1.x * v2.x + v1.y * v2.y);
}

public float distance(Vector v1, Vector v2){
  return sqrt(((v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y)));
}

public float distanceSqr(Vector v1, Vector v2){
  return ((v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y));
}

Vector cross(Vector v, float z){
  return new Vector(-v.y * z, v.x * z);
}

float cross(Vector v1, Vector v2){
  return v1.x * v2.y - v1.y * v2.x;
}

//https://en.wikipedia.org/wiki/Triple_product
//https://github.com/dyn4j/dyn4j/blob/master/src/main/java/org/dyn4j/geometry/Vector2.java op regel 210
Vector tripleProduct(Vector a, Vector b, Vector c){
  Vector r = new Vector();
  float dot = a.x * b.y - b.x * a.y;
  r.x = -c.y * dot;
  r.y = c.x * dot;
  return r;
}


public Vector randomVector(float min, float max){
    return(new Vector(random(min, max), random(min, max)));
}

//Methods for printing
public void print(Vector v){
  print(v.x + ", " + v.y); 
}

public void println(Vector v){
  println(v.x + ", " + v.y); 
}

public void println(Vector[] array){
  println("[");
  for(Vector v : array){
    println("(" + v.x + ", " + v.y + ")");
  }
  println("]");
}

public void println(ArrayList<Vector> list){
  println("[");
  for(Vector v : list){
    println("(" + v.x + ", " + v.y + ")");
  }
  println("]");
}

public void line(Vector v){
  line(0, 0, v.x, v.y);
}


float round(float x, int n){ //(number to round, number of decimals)   
  return (float)(floor(x * pow(10, n))/pow(10, n));
}