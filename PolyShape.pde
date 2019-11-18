class PolyShape{
  
  Vector pos;
  Vector vel;
  Vector acc;
  float aPos;
  float aVel;
  float aAcc;
  
  float radius;
  float mass;
  float moi;
  
  Vector[] originalVertices;
  Vector[] localVertices;
  Vector[] worldVertices;
  
  color defColor = color(0, 200, 100);
  
  boolean overlapping;
  color overlapColor = color(255, 0, 0);
  
  boolean movable = true;
  boolean rotatable = true;
  boolean showDirection = true;
  
  
  PolyShape(Vector[] localVertices, Vector pos, float mass){
    this.pos = new Vector(pos.x, pos.y);
    this.vel = new Vector();
    this.acc = new Vector();
    this.aPos = 0;
    this.aVel = 0;
    this.aAcc = 0;
    
    this.mass = mass;
    
    this.originalVertices = new Vector[localVertices.length];
    for(int i = 0; i < localVertices.length; i++){
      this.originalVertices[i] = localVertices[i].copy();
    }
    this.localVertices = localVertices;
    this.worldVertices = computeWorldVertices();
    
    // Only for regular polygons
    this.radius = dist(0, 0, localVertices[0].x, localVertices[0].y);
    
    // Only works for regular polygons
    this.moi = 0.5 * mass * radius*radius * (1 - ((2/3)*sin(PI/localVertices.length)));
  }
  

  
  void step(float dt){
    // Fake friction
    vel.mult(0.99);
    aVel *= 0.99;
    
    vel.add(mult(acc, dt));
    pos.add(mult(vel, dt));
    acc = new Vector();
    aVel += aAcc * dt;
    aPos += aVel * dt;
    aAcc = 0;
    if(abs(aVel) < 0.001 && aAcc == 0) aVel = 0;
    
    println(aVel, aPos);
    transformShape();
    
  }
  
  void render(){
    strokeWeight(2);
    if(overlapping) stroke(overlapColor);
    else stroke(defColor);

    // Draw edges
    for(int i = 0; i < worldVertices.length; i++){
      int i1 = (i + 1) % worldVertices.length; // make sure to wrap around
      line(worldVertices[i].x, worldVertices[i].y, worldVertices[i1].x, worldVertices[i1].y);
    }
    
    if(showDirection){
      // Draw centroid
      fill(200, 0, 0);
      stroke(200, 0, 0);
      ellipse(pos.x, pos.y, 4, 4);
      
      // Draw direction
      strokeWeight(2);
      stroke(200, 0, 0);
      line(pos.x, pos.y, worldVertices[0].x, worldVertices[0].y);
    }
  }
  
  void transformShape(){
    float cosine = cos(aPos);
    float sine = sin(aPos);
    
    for(int i = 0; i < worldVertices.length; i++){
      //println(originalVertices[i]);
      Vector v = originalVertices[i];
      //Vector v = new Vector(originalVertices[i].x, originalVertices[i].y);
      //println(x + ", " + y);
      
      localVertices[i].x = ((v.x * cosine) + (v.y * -sine ));
      localVertices[i].y = ((v.x * sine  ) + (v.y * cosine));
      worldVertices[i] = add(localVertices[i], pos);
      // Scale shape
    }
  }
  
  Vector[] computeWorldVertices(){
    Vector[] worldVertices = new Vector[localVertices.length];
    for(int i = 0; i < localVertices.length; i++){
      worldVertices[i] = add(localVertices[i], pos);
    }
    return worldVertices;
  }
  
  // Get the vertex which is furthest along the given direction vector
  Vector getSupport(Vector direction){
    float furthestDistance = -999999;
    Vector furthestVertex = new Vector();
    
    for(int i = 0; i < localVertices.length; i++){
      float distance = dot(localVertices[i], direction);
      if(distance > furthestDistance){
        furthestDistance = distance;
        furthestVertex = worldVertices[i];
      }
    }
    
    return furthestVertex;
  }
  
  int getSupportIndex(Vector direction, Vector supportVertex){
    float furthestDistance = -999999;
    int furthestVertexIndex = -1;
    
    for(int i = 0; i < localVertices.length; i++){
      float distance = dot(localVertices[i], direction);
      if(distance > furthestDistance){
        furthestDistance = distance;
        supportVertex.x = worldVertices[i].x;
        supportVertex.y = worldVertices[i].y;
        furthestVertexIndex = i;
      }
    }
    
    return furthestVertexIndex;
  }
  
  int getVertexCount(){
    return localVertices.length;  
  }
  
  void applyForceAndTorque(Vector f, Vector p){
    // Apply force
    // Make the force weaker the farther from the center of mass it was applied (inverse of torque)
    //println(f.mag());
    //float magSqr = f.magSqr();
    //magSqr = constrain(magSqr, 0, 100*100);
    float dist = sub(p, pos).mag();
    //println(dist);
    float ratio = 1 - (dist / radius/2);
    ratio = constrain(ratio, 0.0, 1.0);
    //println(ratio);
    Vector force = mult(f, ratio);

    float multiplier = 200; // Purely to make it look a little more realistic
    force.mult(-multiplier);
    //println(force.mag());
    acc = div(force, mass);
    
    // Apply torque
    Vector r = sub(pos, p);
    float T = r.x * f.y - r.y * f.x;
    T *= multiplier;
    aAcc = T / moi;
  }
 
  
}

// Calculate the center off the shape
Vector computeCentroid(Vector[] vertices){
  Vector centroid = new Vector();
  float det = 0;
  float tempDet = 0;
  int j = 0;
  
  for(int i = 0; i < vertices.length; i++){
    if(i + 1 == vertices.length){
      j = 0;
    }
    else{
      j = i + 1;
    }
    tempDet = vertices[i].x * vertices[j].y - vertices[j].x*vertices[i].y;
    det += tempDet;
    centroid.x += (vertices[i].x + vertices[j].x)*tempDet;
    centroid.y += (vertices[i].y + vertices[j].y)*tempDet;
  }
  
  centroid.x /= 3*det;
  centroid.y /= 3*det;
  
  return centroid;
}

Vector[] generateRegularPolyVerts(float r, int n){
  Vector[] vertices = new Vector[n];
  
  // Wound anti clockwise
  for(int i = 0; i < n; i++){
    float da = 2*PI - ((2*PI / n) * i);
    vertices[i] = new Vector(cos(da) * r, sin(da) * r);
  }
  
  return vertices;
}