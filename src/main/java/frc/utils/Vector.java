package frc.utils;

import java.util.function.Consumer;
//a class representing either a vector or a Point in space in 2 or more dimensions with double precision
//with operations for manipulating them

//most operations affect the point they're called on, so for non-destructive operations, use .copy()

//when clockwise or counterclockwise is mentioned, keep in mind that in java, positive y is down, not up
public class Vector {
	//the array of values for the location of the Point,
	//from lowest dimension to highest, 
	//ie. x-value is vals[0], y-value is vals[1], etc.
	private double[] m_vals;
	
	//constructs a new 2d Point at (0, 0)
	public Vector() {
		this(0, 0);
	}
	
	//constructs a new 2d Point at (x, y)
	public Vector(double x, double y) {
		m_vals = new double[2];
		m_vals[0] = x;
		m_vals[1] = y;
	}
	
	//constructs a new 3d Point at (x, y, z)
	public Vector(double x, double y, double z) {
		m_vals = new double[3];
		m_vals[0] = x;
		m_vals[1] = y;
		m_vals[2] = z;
	}
	
	//constructs a Point with the given array of values as it'a location,
	//with the number of dimensions equal to the length of the array
	public Vector(double[] vals) {
		if(vals.length < 2) {
			throw new IllegalArgumentException("dimension counts less than 2 not supported");
		}
		this.m_vals = vals;
	}
	
	//constructs a Point with the given number of dimensions, with each position as 0
	public Vector(int dimensions) {
		if(dimensions < 2) {
			throw new IllegalArgumentException("dimension counts less than 2 not supported");
		}
		m_vals = new double[dimensions];
	}
	
	//returns a copy of this Point
	public Vector copy() {
		Vector newP = new Vector(m_vals.length);
		for(int i = 0; i < m_vals.length; i++) {
			newP.set(i, m_vals[i]);
		}
		return newP;
	}
	
	//adds another dimension to this Point, initializing the value of that dimension to 0
	public Vector addDim() {
		double[] oldVals = m_vals;
		m_vals = new double[m_vals.length + 1];
		for(int i = 0; i < oldVals.length; i++) {
			m_vals[i] = oldVals[i];
		}
		m_vals[oldVals.length] = 0;
		return this;
	}
	
	//returns the number of dimensions of the Point
	public int dims() {
		return m_vals.length;
	}
	
	//returns a String representation of this Point
	//in the format (x,y) for 2 dimensions, (x,y,z) for 3 dimensions, etc.
	@Override
	public String toString() {
		String s = "(" + m_vals[0];
		for(int i = 1; i < m_vals.length; i++) {
			s += "," + m_vals[i];
		}
		return s + ")";
	}
	
	//constructs a new Point from a string in the same format as returned by the toString() method
	public static Vector fromString(String data) {
		data = data.substring(1, data.length() - 1);
		String[] valStrings = data.split(",");
		double[] vals = new double[valStrings.length];
		for(int i = 0; i < valStrings.length; i++) vals[i] = Double.parseDouble(valStrings[i]);
		return new Vector(vals);
	}
	
	//returns the distance from the origin of this Point, of it's length as a vector
	public double mag() {
		double n = 0;
		for(double d : m_vals) {
			n += d * d;
		}
		return Math.sqrt(n);
	}
	
	//returns the distance from the origin of this point squared, and runs faster than mag()
	public double sMag() {
		double n = 0;
		for(double d : m_vals) {
			n += d * d;
		}
		return n;
	}
	
	//return this Point's x-value
	public double X() {
		return m_vals[0];
	}
	
	//sets this Point's x-value to n
	public void setX(double n) {
		m_vals[0] = n;
	}
	
	//return this Point's y-value
	public double Y() {
		return m_vals[1];
	}
	
	//sets this Point's y-value to n
	public void setY(double n) {
		m_vals[1] = n;
	}
	
	//return this Point's z-value if it has one
	public double Z() {
		if(m_vals.length < 2) throw new IllegalStateException("z-value requires a point with at least 3 dimensions");
		return m_vals[2];
	}
	
	//sets this Point's z-value to n if it has a z-value
	public void setZ(double n) {
		if(m_vals.length < 2) throw new IllegalStateException("z-value requires a point with at least 3 dimensions");
		m_vals[2] = n;
	}
	
	//gets the value of dimension dim of this Point, where dimension 0 is x, dimension 1 is y, etc.
	public double get(int dim) {
		if(m_vals.length <= dim) {
			return 0;
		}
		return m_vals[dim];
	}
	
	//gets the value of dimension dim of this Point to n, where dimension 0 is x, dimension 1 is y, etc.
	public void set(int dim, double n) {
		m_vals[dim] = n;
	}
	
	//gets the distance between this Point and Point p
	public double dist(Vector p) {
		if(p.m_vals.length != this.m_vals.length) {
			throw new IllegalArgumentException("points to compare must have the same number of dimensions");
		}
		return p.copy().sub(this).mag();
	}
	
	//sets this point to a vector of length 1 going in the same direction
	//then returns this point
	public Vector normalize() {
		div(mag());
		return this;
	}
	
	//divides all this Point's values by d
	//returns this Point
	public Vector div(double d) {
		for(int i = 0; i < m_vals.length; i++) {
			m_vals[i] /= d;
		}
		return this;
	}
	
	//multiplies all this Point's values by double d 
	//returns this Point
	public Vector mult(double d) {
		for(int i = 0; i < m_vals.length; i++) {
			m_vals[i] *= d;
		}
		return this;
	}
	
	//multiplies all this Point's values by long l 
	//returns this Point
	public Vector mult(long l) {
		for(int i = 0; i < m_vals.length; i++) {
			m_vals[i] *= l;
		}
		return this;
	}
	
	//subtracts Point p from this Point. if p has a different number of dimensions,
	//then only the shared dimensions are subtracted
	public Vector sub(Vector p) {
		for(int i = 0; i < Math.min(m_vals.length, p.m_vals.length); i++) {
			this.m_vals[i] -= p.m_vals[i];
		}
		return this;
	}
	
	//subtracts x from the x-value of this Point, and y from the y-value of this Point
	public Vector sub(double x, double y) {
		m_vals[0] -= x;
		m_vals[1] -= y;
		return this;
	}
	
	//returns the dot product between this Point and Point p
	//if one of the Points is normalized, this can be thought of as getting
	//the other Point's distance along the axis along that Point
	//throws an IllegalArgumentException if the Points have different numbers of dimensions
	//if both Points are normalized, this can be used to get the cosine of the angle between them
	public double dot(Vector p) {
		if(p.m_vals.length != this.m_vals.length) {
			throw new IllegalArgumentException("dot product requires the same number of dimensions between points");
		}
		double sum = 0;
		for(int i = 0; i < m_vals.length; i++) {
			sum += this.m_vals[i] * p.m_vals[i];
		}
		return sum;
	}
	
	//returns the dot product between this Point and Point p using only the x and y dimensions
	//if one of the Points is normalized, this can be thought of as getting
	//the other Point's distance along the axis along that Point
	//if both Points are normalized, this can be used to get the cosine of the angle between them
	public double dot2d(Vector p) {
		double sum = 0;
		for(int i = 0; i < 2; i++) {
			sum += this.m_vals[i] * p.m_vals[i];
		}
		return sum;
	}
	
	//sets all of this Point's values to the absolute of that value
	//returns this Point
	public Vector abs() {
		for(int i = 0; i < m_vals.length; i++) {
			m_vals[i] = Math.abs(m_vals[i]);
		}
		return this;
	}

    //restricts this Point to a maximum length, setting it to that length it it 
    //it longer, then returns itself
    public Vector clampLength(double maxLength) {
        if(this.sMag() > maxLength * maxLength) {
            normalize();
            mult(maxLength);
        }
        return this;
    }
	
	//adds Point p to this Point. if p has a different number of dimensions to this one,
	//only the shared dimensions are added.
	//returns this Point
	public Vector add(Vector p) {
		if(p.m_vals.length > this.m_vals.length) {
			throw new IllegalArgumentException("points to add must have the same number of dimensions or less");
		}
		for(int i = 0; i < Math.min(m_vals.length, p.m_vals.length); i++) {
			this.m_vals[i] += p.m_vals[i];
		}
		return this;
	}
	
	//adds x to the x-value of this Point, and y to the y-value of this Point
	public Vector add(double x, double y) {
		m_vals[0] += x;
		m_vals[1] += y;
		return this;
	}
	
	//applies a 2d linear transformation to this Point, where the transformed location of (1, 0)
	//is iHatLoc, and the transformed location of (0, 1) is jHatLoc, and is also equivalent to
	//multiplying the matrix with iHatLoc and jHatLoc as it's columns by this Vector
	//returns this Point
	public Vector matrixTransform(Vector iHatLoc, Vector jHatLoc) {
		double newX = Y() * jHatLoc.X() + X() * iHatLoc.X();
		double newY = Y() * jHatLoc.Y() + X() * iHatLoc.Y();
		setX(newX);
		setY(newY);
		return this;
	}
	
	//multiplies this Point by Point o as if they are complex numbers in the
	//format x + yi, then returns this Point.
	//throws an IllegalArgumentException if either Point has more than 2 dimensions
	public Vector cMult(Vector o) {
		if(o.m_vals.length > 2 || this.m_vals.length > 2) {
			throw new IllegalArgumentException();
		}
		double newX = X() * o.X() - Y() * o.Y();
		double newY = X() * o.Y() + Y() * o.X();
		setX(newX);
		setY(newY);
		return this;
	}
	
	//divides this Point by Point o as if they are complex numbers in the
	//format x + yi, then returns this Point.
	//throws an IllegalArgumentException if either Point has more than 2 dimensions
	public Vector cDiv(Vector o) {
		double oSqrd = o.X() * o.X() + o.Y() * o.Y();
		double newX = (X() * o.X() + Y() * o.Y()) / oSqrd;
		double newY = (Y() * o.X() - X() * o.Y()) / oSqrd;
		setX(newX);
		setY(newY);
		return this;
	}
	
	//returns a new Point rotated 90 degrees counterclockwise
	//around the origin from this one, with the same magnitude
	public Vector getPerpendicular() {
		return new Vector(-Y(), X());
	}
	
	//rotates this Point rot degrees around the origin clockwise,
	//then returns this Point
	public Vector rot(double rot) {
		Vector newXLoc = new Vector(Math.cos(rot), -Math.sin(rot));
		Vector newYLoc = newXLoc.getPerpendicular();
		return matrixTransform(newXLoc, newYLoc);
	}
	
	//clamps the value of a Point between the limits given by minimum and maximum Points such that 
	//min.X() <= max.X() and min.Y() <= max.Y()
	public Vector clamp(Vector min, Vector max) {
		if(min.dims() != max.dims()) {
			throw new IllegalArgumentException("arguments must have the same number of dimensions");
		}
		for(int i = 0; i < dims() && i < min.dims(); i++) {
			if(m_vals[i] < min.m_vals[i]) m_vals[i] = min.m_vals[i];
			if(m_vals[i] > max.m_vals[i]) m_vals[i] = max.m_vals[i];
		}
		return this;
	}
	
	//transforms this Point into the space of Point space, then returns this Point
	//this Point is also multiplied by the magnitude of space in the process, 
	//so if you want to avoid this, normalize space first
	public Vector toSpace(Vector space) {
		double oldX = X();
		double oldY = Y();
		setX(oldX * space.X() + oldY * space.Y());
		setY(oldY * space.X() - oldX * space.Y());
		return this;
	}
	
	//returns whether this Point is equal to Point p
	//only returns true if they have the same number of dimensions,
	//and all values are equal between them
	public boolean equals(Vector p) {
		if(p.m_vals.length != this.m_vals.length) {
			return false;
		}
		for(int i = 0; i < m_vals.length; i++) {
			if(this.m_vals[i] != p.m_vals[i]) {
				return false;
			}
		}
		return true;
	}
	
	//returns the angle from (0, 0) to this Point
	public double ang() {
		return Math.atan2(Y(), X());
	}
	
	//accepts this Point with Consumer<Point> c
	public void transform(Consumer<Vector> c) {
		c.accept(this);
	}
	
	//returns whether this Point is within the given minimum and maximum bounds
	public boolean isWithinBounds(Vector boundsMin, Vector boundsMax) {
		double minX;
		double maxX;
		double minY;
		double maxY;
		
		if(boundsMin.X() < boundsMax.X()) {
			minX = boundsMin.X();
			maxX = boundsMax.X();
		} else {
			maxX = boundsMin.X();
			minX = boundsMax.X();
		}
		
		if(boundsMin.Y() < boundsMax.Y()) {
			minY = boundsMin.Y();
			maxY = boundsMax.Y();
		} else {
			maxY = boundsMin.Y();
			minY = boundsMax.Y();
		}
		
		return minX <= X() && X() <= maxX && minY <= Y() && Y() <= maxY;
	}
}