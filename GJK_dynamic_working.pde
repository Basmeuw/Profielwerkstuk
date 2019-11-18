ArrayList<PolyShape> shapes = new ArrayList<PolyShape>();
float elasticity = 1;
float prevMillis = millis();
String collisionRespone = "linear"; // either rotation or linear


void setup(){
  size(1280, 720);
  
  PolyShape p;
  Vector[] pv = generateRegularPolyVerts(70, 4);
  //pv[0].x += 50;
  Vector ppos = new Vector(width/2 - 100, height/2);
  p = new PolyShape(pv, ppos, 1); 
  shapes.add(p);
  
  PolyShape p1;
  Vector[] p1v = generateRegularPolyVerts(70, 6);
  p1v[0].x += 100;
  Vector p1pos = new Vector(width/2 + 200, height/2);
  p1 = new PolyShape(p1v, p1pos, 1);
  p1.aPos = PI;
  shapes.add(p1);
  
  //createBorder();
  
}


void draw(){
  background(51);
  
  println((millis() - prevMillis)/1000);
  stepSimulation((millis() - prevMillis)/1000);
  prevMillis = millis();
}

void stepSimulation(float deltaTime){
  
  //shapes.get(0).pos = new Vector(mouseX, mouseY);
  
  for(PolyShape s : shapes){
    s.overlapping = false;
  }
  
  for(PolyShape s : shapes){

    s.step(deltaTime);
    
  }

  //dir = sub(new Vector(mouseX, mouseY), new Vector(width/2, height/2));
  
  // COLLISION
  for(int i = 0; i < shapes.size(); i++){
    for(int j = i+1; j < shapes.size(); j++){
      GJK2D gjk = new GJK2D(shapes.get(i), shapes.get(j));
      Vector penetration = gjk.intersect();
      
      boolean collided = gjk.collided;
      
      if(collided){
        Vector contact = gjk.getContactPoint(penetration);
        if(contact != null){
        
          //pushMatrix();
          //translate(width/2, height/2);
          //penetration.display();
          //popMatrix();
          
          
          
          PolyShape A = shapes.get(i);
          PolyShape B = shapes.get(j);
          
          // https://sidvind.com/wiki/2D-Collision_response
          if(collisionRespone.equals("linear")){
            Vector velAB = sub(B.vel, A.vel);
            //Vector normal = normalize(new Vector(-penetration.y, penetration.x));
            Vector normal = normalize(penetration);
            
            Vector a = sub(velAB.neg(), mult(velAB, elasticity));
            float jnumerator = dot(a, normal);
            
            float b = dot(normal, normal);
            float jdenominator = (b/A.mass) + (b/B.mass);
            
            float J = jnumerator / jdenominator;
            
            if(A.movable) A.vel.sub(mult(normal, J/A.mass));
            if(B.movable) B.vel.add(mult(normal, J/B.mass));
            
          }else if(collisionRespone.equals("rotation")){
            Vector velAB = sub(B.vel, A.vel);
            Vector normal = normalize(penetration);
            // Vector pointing from the center to the contact
            Vector rap = sub(contact, A.pos);
            Vector rbp = sub(contact, B.pos);
            
            Vector a = sub(velAB.neg(), mult(velAB, elasticity));
            float jnumerator = dot(a, normal);
            
            float b = dot(normal, normal);
            float jdenominator1 = (b/A.mass) + (b/B.mass);
            
            // (a x b)^2 = (a x b) dot (a x b) = (a x b) x a dot b
            
            float ca = pow(dot(rap, normal), 2) / A.moi;
            float cb = pow(dot(rbp, normal), 2) / B.moi;
            
            float jdenominator = jdenominator1 + ca + cb;
            
            float J = jnumerator/jdenominator;
            
            // Move linear
            if(A.movable) A.vel.sub(mult(normal, J/A.mass));
            if(B.movable) B.vel.add(mult(normal, J/B.mass));
            
            // Rotate
            if(A.rotatable) A.aVel -= cross(rap, mult(normal, J)) / A.moi;
            if(B.rotatable) B.aVel += cross(rbp, mult(normal, J)) / A.moi;
          }
          
          if(A.movable && B.movable){
            A.pos.sub(div(penetration, 2));
            B.pos.add(div(penetration, 2));
          }else if(!B.movable){
            A.pos.sub(penetration);
          }else if(!A.movable){
            B.pos.add(penetration);
          }
          
        }
      }
      
      if(collided){
        gjk.shapeA.overlapping = true;
        gjk.shapeB.overlapping = true;
      }else{
        gjk.shapeA.overlapping |= false;
        gjk.shapeB.overlapping |= false;
      }
    }
  }
  
  
  renderSimulation();
  
  
}

void renderSimulation(){
  // Reset screen
  
  // Render force arrow
  if(hasPressedBefore){
    stroke(255, 0, 0);
    strokeWeight(2);
    fill(255, 0, 0);
    ellipse(firstPress.x, firstPress.y, 3, 3);
    line(firstPress.x, firstPress.y, mouseX, mouseY);
  }
  
  // Render shapes
  for(PolyShape s : shapes){
    s.render();
  }
}

boolean hasPressedBefore = false;
boolean stateChangedBefore = false;
Vector firstPress;
Vector secondPress;
PolyShape selectedShape;
void mousePressed(){
  stateChangedBefore = false;
  if(mouseButton == RIGHT){
    if(hasPressedBefore){
      hasPressedBefore = false;
      secondPress= new Vector(mouseX, mouseY);
      Vector f = sub(secondPress, firstPress);
      selectedShape.applyForceAndTorque(f, firstPress);
      selectedShape = null;
      stateChangedBefore = true;
    }
  }else if(mouseButton == LEFT){
    hasPressedBefore = false;
  }

  for(PolyShape s : shapes){
    float dist = dist(mouseX, mouseY, s.pos.x, s.pos.y);
    if(mouseButton == RIGHT && !stateChangedBefore){
      if(dist <= s.radius){
        if(!hasPressedBefore){
          hasPressedBefore = true;
          //println(hasPressedBefore);
          firstPress = new Vector(mouseX, mouseY);
          selectedShape = s;
        }
      } 
    }
  }
}

void createBorder(){
  PolyShape borderleft;
  Vector[] blv = new Vector[4];
  blv[0] = new Vector(0, 0);
  blv[1] = new Vector(width, 0);
  blv[2] = new Vector(width, height);
  blv[3] = new Vector(0, height);
  borderleft = new PolyShape(blv, blv[0], 1);
  borderleft.movable = false;
  borderleft.rotatable = false;
  shapes.add(borderleft);
}