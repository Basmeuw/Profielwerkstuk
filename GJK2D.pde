class GJK2D{
  
  PolyShape shapeA;
  PolyShape shapeB;
  
  ArrayList<Vector> simplex;
  
  boolean collided = false;
  
  Vector direction = new Vector();
  
  // The last index is always the last added point (A)
  
  GJK2D(PolyShape shapeA, PolyShape shapeB){
    this.shapeA = shapeA;
    this.shapeB = shapeB;
    
    simplex = new ArrayList<Vector>();
  }
  
  boolean addSupport(Vector direction){
    Vector support = sub(shapeA.getSupport(direction), shapeB.getSupport(direction.neg()));
    simplex.add(support);
    return dot(support, direction) >= 0;
  }
  
  
  
  // Return 0 means still evolving
  // Return 1 means no intersection
  // Return 2 means found intersecion

  int evolveSimplex(){
    //println(simplex.size());
    switch(simplex.size()){
      case 0:{
        direction = sub(shapeB.pos, shapeA.pos);
        break;
      }
      case 1:{
        direction = sub(shapeA.pos, shapeB.pos); // Flip direction
        break;
      }
      case 2:{
        // Add the last vertex to the simplex
        Vector b = simplex.get(1);
        Vector c = simplex.get(0);
        // Vector of the first two verts
        Vector cb = sub(b, c);
        // Vector from the first vert to the origin
        Vector co = mult(c, -1);
        
        // Calculate direction perpendicular to b and c in the dir of the origin
        direction = tripleProduct(cb, co, cb);
        break;
      }
      case 3:{
        // When the simplex is full
        // Determine if the simplex contains the origin
        // If not, remove te last vertex, and add a new one
        
        Vector a = simplex.get(2);
        Vector b = simplex.get(1);
        Vector c = simplex.get(0);
        
        Vector ao = mult(a, -1);
        Vector ab = sub(b, a);
        Vector ac = sub(c, a);
        
        //Normal of ab in the dir of the origin
        Vector abPerp = tripleProduct(ac, ab, ab);
        Vector acPerp = tripleProduct(ab, ac, ac);
        
        if(abPerp.dot(ao) > 0){
          simplex.remove(c);
          direction = abPerp;
        }else if(acPerp.dot(ao) > 0){
          simplex.remove(b);
          direction = acPerp;
        }else{
          collided = true;
          return 2;
        }
        break;
        
      }
    
    }
    boolean isPastOrigin = addSupport(direction);
    if(isPastOrigin) return 0;
    else return 1;
    

  }
  
  boolean detect(){
    int result = 0;
    while(result == 0){
      result = evolveSimplex();
    }
    //println(result);
    return result == 2;
  }
  
  Edge findClosestEdge(){
    float closestDistance = 999999;
    Vector closestNormal = new Vector();
    int closestIndex = 0;
    Vector edge = new Vector();
    
    for(int i = 0; i < simplex.size(); i++){
      int j = (i + 1) % simplex.size(); // index of the next vertex
      
      edge = sub(simplex.get(j), simplex.get(i));
      
      Vector normal = new Vector(-edge.y, edge.x);
      normal.normalize();
      
      float distance = normal.dot(simplex.get(i));
      
      if(distance < closestDistance){
        closestDistance = distance;
        closestNormal = normal;
        closestIndex = j;
      }
    }
    return new Edge(closestDistance, closestNormal, closestIndex);
  }
  
  Vector intersect(){
    //getContactManifold();
    if(!detect()){
      return null;
    }
    
    Vector intersection = new Vector();
    for(int i = 0; i < 32; i++){
      Edge edge = findClosestEdge();
      Vector support = sub(shapeA.getSupport(edge.normal), shapeB.getSupport(edge.normal.neg()));
      float distance = dot(support, edge.normal);
      
      intersection = edge.normal.copy();
      intersection.mult(distance);
      
      if(abs(distance - edge.distance) <= 0.0000001){
        return intersection;
      }else{
        simplex.add(edge.index, support);
      }
    }
    return intersection;
    
  }
  
  // Calculates the contact manifold
  // This assumes a collision was found
  ArrayList<Vector> getContactManifold(){
    
    // First get the edges which could actually be in isct
    Vector closestVertexA = new Vector();
    Vector closestVertexB = new Vector();
    
    Vector dir = normalize(sub(shapeB.pos, shapeA.pos));
    int closestVertexIndexA = shapeA.getSupportIndex(dir, closestVertexA);
    int closestVertexIndexB = shapeB.getSupportIndex(mult(dir, -1), closestVertexB);

    // Get the indexes off the vertices we need
    int iA1 = closestVertexIndexA;
    int iA0 = (iA1 == 0) ? shapeA.getVertexCount() - 1 : iA1 - 1;
    int iA2 = (iA1 == shapeA.getVertexCount() - 1) ? 0 : iA1 + 1;
    
    int iB1 = closestVertexIndexB;
    int iB0 = (iB1 == 0) ? shapeB.getVertexCount() - 1 : iB1 - 1;
    int iB2 = (iB1 == shapeB.getVertexCount() - 1) ? 0 : iB1 + 1;
   
    
    // Now do intersection test between each of the lines
    ArrayList<Vector> contactPoints = new ArrayList<Vector>();
    // A01 vs B01
    Vector point0 = lineLine(shapeA.worldVertices[iA1], shapeA.worldVertices[iA0], shapeB.worldVertices[iB1], shapeB.worldVertices[iB0]);
    // A01 vs B21
    Vector point1 = lineLine(shapeA.worldVertices[iA1], shapeA.worldVertices[iA0], shapeB.worldVertices[iB1], shapeB.worldVertices[iB2]);
    // A21 vs B01
    Vector point2 = lineLine(shapeA.worldVertices[iA1], shapeA.worldVertices[iA2], shapeB.worldVertices[iB1], shapeB.worldVertices[iB0]);
    // A21 vs B21
    Vector point3 = lineLine(shapeA.worldVertices[iA1], shapeA.worldVertices[iA2], shapeB.worldVertices[iB1], shapeB.worldVertices[iB2]);
    
    // Check if there was a isct for each test
    if (point0 != null) contactPoints.add(point0);
    if (point1 != null) contactPoints.add(point1);
    if (point2 != null) contactPoints.add(point2);
    if (point3 != null) contactPoints.add(point3);
    
    for(Vector cp : contactPoints){
      fill(0, 0, 200);
      noStroke();
      ellipse(cp.x, cp.y, 12, 12);
    }
    
    return contactPoints;
  }
  
  // This assumes a collision was found
  Vector getContactPoint(Vector intersection){
    // This uses a trick
    // It moves the polygons 90% away from each other
    // Then it calculates the cps and averages them to one point
    
    Vector partialIntersection = mult(intersection, 0.99);
    
    // First get the edges which could actually be in isct
    Vector closestVertexA = new Vector();
    Vector closestVertexB = new Vector();
    
    Vector dir = normalize(sub(shapeB.pos, shapeA.pos));
    int closestVertexIndexA = shapeA.getSupportIndex(dir, closestVertexA);
    int closestVertexIndexB = shapeB.getSupportIndex(mult(dir, -1), closestVertexB);

    // Get the indexes off the vertices we need
    int iA1 = closestVertexIndexA;
    int iA0 = (iA1 == 0) ? shapeA.getVertexCount() - 1 : iA1 - 1;
    int iA2 = (iA1 == shapeA.getVertexCount() - 1) ? 0 : iA1 + 1;
    
    int iB1 = closestVertexIndexB;
    int iB0 = (iB1 == 0) ? shapeB.getVertexCount() - 1 : iB1 - 1;
    int iB2 = (iB1 == shapeB.getVertexCount() - 1) ? 0 : iB1 + 1;
    
    // All of these values are translated by 90% of the intersection vector
    Vector A0translated = add(shapeA.worldVertices[iA0], partialIntersection);
    Vector A1translated = add(shapeA.worldVertices[iA1], partialIntersection);
    Vector A2translated = add(shapeA.worldVertices[iA2], partialIntersection);
    
    Vector B0translated = sub(shapeB.worldVertices[iB0], partialIntersection);
    Vector B1translated = sub(shapeB.worldVertices[iB1], partialIntersection);
    Vector B2translated = sub(shapeB.worldVertices[iB2], partialIntersection);
    
    // Now do intersection test between each of the lines
    ArrayList<Vector> contactPoints = new ArrayList<Vector>();
    // A01 vs B01
    Vector point0 = lineLine(A1translated, A0translated, B1translated, B0translated);
    // A01 vs B21
    Vector point1 = lineLine(A1translated, A0translated, B1translated, B2translated);
    // A21 vs B01
    Vector point2 = lineLine(A1translated, A2translated, B1translated, B0translated);
    // A21 vs B21
    Vector point3 = lineLine(A1translated, A2translated, B1translated, B2translated);
    
    // Check if there was a isct for each test
    if (point0 != null) contactPoints.add(point0);
    if (point1 != null) contactPoints.add(point1);
    if (point2 != null) contactPoints.add(point2);
    if (point3 != null) contactPoints.add(point3);
    
    //for(Vector cp : contactPoints){
    //  fill(0, 0, 200);
    //  noStroke();
    //  ellipse(cp.x, cp.y, 12, 12);
    //}
    Vector cp = null;
    if(contactPoints.size() == 2){
      println("cp = 2");
      // Calculate the average
      cp = new Vector();
      cp.x = (contactPoints.get(0).x + contactPoints.get(1).x) / 2;
      cp.y = (contactPoints.get(0).y + contactPoints.get(1).y) / 2;
    }else if (contactPoints.size() ==  1){
      println("cp = 1");
      cp = contactPoints.get(0);
    }else{
      println("cp = 0");
    }
     
    return cp;
  }
  
  Vector lineLine(Vector a, Vector b, Vector c, Vector d){
    return lineLine(a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y);
  }
  
  Vector lineLine(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4){
    // All formulas can be found on
    // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    float noemer = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  
    // If denominator == 0 then the lines run parrallel.
    if(noemer == 0){
      return null;
    }
    
    // Calculate how far we are along the first line segment
    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / noemer;
    // Calculate how far along the second line we are
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / noemer;
    
    // Now comparing for 1 line seg and one infinite line
    // If you want to compare two line segs,
    // Add "&& u < 1" to the if statement
    if(t > 0 && t < 1 && u > 0 && u < 1){
      Vector point = new Vector(x1 + (t * (x2 - x1)), y1 + (t * (y2 - y1)));
      return point;
    }else{
      return null;
    }
  }
}

// Class for the closest Edge
class Edge{
  float distance;
  Vector normal;
  int index;
  
  Edge(float cd, Vector cn, int ci){
    distance = cd;
    normal = cn;
    index = ci;
  }
}