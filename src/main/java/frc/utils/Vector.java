package frc.utils;

// a class representing either a vector or a Point in space in 2 or more dimensions with double
// precision
// with operations for manipulating them

public class Vector {
  public static final Vector Origin = new Vector(0, 0);

  // the array of values for the location of the Point,
  // from lowest dimension to highest,
  // ie. x-value is vals[0], y-value is vals[1], etc.
  protected double[] m_vals;

  /** constructs a new 2d Vector at (0,0) */
  public Vector() {
    this(0, 0);
  }

  /**
   * constructs a new 2d Vector at (x, y)
   *
   * @param x the x value for the Vector
   * @param y the y value for the Vector
   */
  public Vector(double x, double y) {
    m_vals = new double[2];
    m_vals[0] = x;
    m_vals[1] = y;
  }

  /**
   * constructs a new 3d Vector at (x, y, z)
   *
   * @param x the x value for the Vector
   * @param y the y value for the Vector
   * @param z the z value for the Vector
   */
  public Vector(double x, double y, double z) {
    m_vals = new double[3];
    m_vals[0] = x;
    m_vals[1] = y;
    m_vals[2] = z;
  }

  /**
   * constructs a new n-dimensional Vector at (vals[0], vals[1], ..., vals[n]) where n is the final
   * element of vals
   *
   * @param vals the dimensions to use for the new Vector
   */
  public Vector(double[] vals) {
    if (vals.length < 2) {
      throw new IllegalArgumentException("dimension counts less than 2 not supported");
    }
    this.m_vals = vals;
  }

  /**
   * constructs a new n-dimensional Vector where all dimensions start as 0
   *
   * @param dimensions the number of dimensions to construct the Vector with
   */
  public Vector(int dimensions) {
    if (dimensions < 2) {
      throw new IllegalArgumentException("dimension counts less than 2 not supported");
    }
    m_vals = new double[dimensions];
  }

  /**
   * returns a Vector which is a copy of this one
   *
   * @return a copy of this Vector
   */
  public Vector copy() {
    Vector newP = new Vector(m_vals.length);
    for (int i = 0; i < m_vals.length; i++) {
      newP.set(i, m_vals[i]);
    }
    return newP;
  }

  /**
   * returns the number of dimensions of this Vector
   *
   * @return the number of dimensions of this Vector
   */
  public int dims() {
    return m_vals.length;
  }

  @Override
  /** returns a String representation of this Vector in the format (x, y, ..., n) */
  public String toString() {
    String s = "(" + m_vals[0];
    for (int i = 1; i < m_vals.length; i++) {
      s += "," + m_vals[i];
    }
    return s + ")";
  }

  /**
   * returns a new Vector using the String representation of a Vector as returned by
   * Vector.toString()
   *
   * @param data the String representation of a vector
   * @return the new Vector
   */
  public static Vector fromString(String data) {
    data = data.substring(1, data.length() - 1);
    String[] valStrings = data.split(",");
    double[] vals = new double[valStrings.length];
    for (int i = 0; i < valStrings.length; i++) vals[i] = Double.parseDouble(valStrings[i]);
    return new Vector(vals);
  }

  /**
   * returns the magnitude of this Vector
   *
   * @return the magnitude of this Vector
   */
  public double mag() {
    double n = 0;
    for (double d : m_vals) {
      n += d * d;
    }
    return Math.sqrt(n);
  }

  /**
   * returns the magnitude of this Vector squared more quickly than mag()
   *
   * @return the magnitude of this Vector squared
   */
  public double squaredMag() {
    double n = 0;
    for (double d : m_vals) {
      n += d * d;
    }
    return n;
  }

  /**
   * returns the X component of this Vector
   *
   * @return the X component of this Vector
   */
  public double x() {
    return m_vals[0];
  }

  /**
   * sets the X component of this Vector to the given value
   *
   * @param n the new value for the X component of this Vector
   */
  public void setX(double n) {
    m_vals[0] = n;
  }

  /**
   * returns the Y component of this Vector
   *
   * @return the Y component of this Vector
   */
  public double y() {
    return m_vals[1];
  }

  /**
   * sets the Y component of this Vector to the given value
   *
   * @param n the new value for the Y component of this Vector
   */
  public void setY(double n) {
    m_vals[1] = n;
  }

  /**
   * returns the Z component of this Vector
   *
   * @return the Z component of this Vector
   */
  public double z() {
    if (m_vals.length < 2)
      throw new IllegalStateException("z-value requires a point with at least 3 dimensions");
    return m_vals[2];
  }

  /**
   * sets the Z component of this Vector to the given value
   *
   * @param n the new value for the Z component of this Vector
   */
  public void setZ(double n) {
    if (m_vals.length < 2)
      throw new IllegalStateException("z-value requires a point with at least 3 dimensions");
    m_vals[2] = n;
  }

  /**
   * returns the value of dimension dim of this Vector where X is dimension 0, Y is dimension 1, and
   * so on
   *
   * @param dim the dimension to be returned
   * @return the value of dimension dim
   */
  public double get(int dim) {
    if (m_vals.length <= dim) {
      return 0;
    }
    return m_vals[dim];
  }

  /**
   * sets the value of dimension dim of this Vector to n
   *
   * @param dim the dimension to be set
   * @param n the value to set dim to
   */
  public void set(int dim, double n) {
    m_vals[dim] = n;
  }

  /**
   * returns the distance between this Vector and Vector p
   *
   * @param p the Vector to get the distance to
   * @return the distance between this Vector and Vector p
   */
  public double dist(Vector p) {
    if (p.m_vals.length != this.m_vals.length) {
      throw new IllegalArgumentException(
          "points to compare must have the same number of dimensions");
    }
    return p.copy().sub(this).mag();
  }

  /**
   * sets the magnitude of this Vector to 1 while maintaining the relative proportions of each
   * dimension
   *
   * @return this Vector
   */
  public Vector normalize() {
    div(mag());
    return this;
  }

  /**
   * divides all the dimensions of this Vector by the given value
   *
   * @param d the number to divide this Vector by
   * @return this Vector
   */
  public Vector div(double d) {
    for (int i = 0; i < m_vals.length; i++) {
      m_vals[i] /= d;
    }
    return this;
  }

  /**
   * divides all the dimensions of this Vector by the given double
   *
   * @param d the number to multiply by
   * @return this Vector
   */
  public Vector mult(double d) {
    for (int i = 0; i < m_vals.length; i++) {
      m_vals[i] *= d;
    }
    return this;
  }

  /**
   * divides all the dimensions of this Vector by the given long value
   *
   * @param d the number to multiply by
   * @return this Vector
   */
  public Vector mult(long l) {
    for (int i = 0; i < m_vals.length; i++) {
      m_vals[i] *= l;
    }
    return this;
  }

  /**
   * adds all the shared dimensions of another vaector to this one
   *
   * @param p the Vector to be added to this one
   * @return this Vector
   */
  public Vector add(Vector p) {
    if (p.m_vals.length > this.m_vals.length) {
      throw new IllegalArgumentException(
          "points to add must have the same number of dimensions or less");
    }
    for (int i = 0; i < Math.min(m_vals.length, p.m_vals.length); i++) {
      this.m_vals[i] += p.m_vals[i];
    }
    return this;
  }

  /**
   * adds the given values to the x and y dimensions of this Vector
   *
   * @param x the value to add to the x dimension of this Vector
   * @param y the value to add to the y dimension of this Vector
   * @return this Vector
   */
  public Vector add(double x, double y) {
    m_vals[0] += x;
    m_vals[1] += y;
    return this;
  }

  /**
   * subtracts the shared dimensions of another Vector from this Vector
   *
   * @param p the Vector to subtract from this one
   * @return this Vector
   */
  public Vector sub(Vector p) {
    for (int i = 0; i < Math.min(m_vals.length, p.m_vals.length); i++) {
      this.m_vals[i] -= p.m_vals[i];
    }
    return this;
  }

  /**
   * subtracts the given x and y values from the x and y dimensions of this Vector
   *
   * @param x the value to subtract from the x dimension of this Vector
   * @param y the value to subtract from the y value of this Vector
   * @return this Vector
   */
  public Vector sub(double x, double y) {
    m_vals[0] -= x;
    m_vals[1] -= y;
    return this;
  }

  /**
   * returns the dot product between this Vector and another if one vector is normalized, this can
   * be thought of as getting the distance along that vector as an axis if both vectors are
   * normalized this can be used to get the cosine of the angle between the two vectors
   *
   * @param p the Vector to get the dot product from
   * @return this Vector
   */
  public double dot(Vector p) {
    if (p.m_vals.length != this.m_vals.length) {
      throw new IllegalArgumentException(
          "dot product requires the same number of dimensions between points");
    }
    double sum = 0;
    for (int i = 0; i < m_vals.length; i++) {
      sum += this.m_vals[i] * p.m_vals[i];
    }
    return sum;
  }

  /**
   * returns the dot product between this Vector and another using only the X and Y dimensions if
   * one vector is normalized, this can be thought of as getting the distance along that vector as
   * an axis if both vectors are normalized this can be used to get the cosine of the angle between
   * the two vectors
   *
   * @param p the Vector to get the dot product from
   * @return this Vector
   */
  public double dot2d(Vector p) {
    double sum = 0;
    for (int i = 0; i < 2; i++) {
      sum += this.m_vals[i] * p.m_vals[i];
    }
    return sum;
  }

  /**
   * sets all of the dimensions of this Vector to their absolute value
   *
   * @return this Vector
   */
  public Vector abs() {
    for (int i = 0; i < m_vals.length; i++) {
      m_vals[i] = Math.abs(m_vals[i]);
    }
    return this;
  }

  /**
   * applies a 2d linear transformation to this Vector, where the transformed location of (1, 0) is
   * iHatLoc, and the transformed location of (0, 1) is jHatLoc, and is also equivalent to
   * multiplying the matrix with iHatLoc and jHatLoc as it's columns by this Vector
   *
   * @param iHatLoc the transformed location of the x axis basis vector
   * @param jHatLoc the transformed location of the y axis basis vector
   * @return this Vector
   */
  public Vector matrixTransform(Vector iHatLoc, Vector jHatLoc) {
    double newX = y() * jHatLoc.x() + x() * iHatLoc.x();
    double newY = y() * jHatLoc.y() + x() * iHatLoc.y();
    setX(newX);
    setY(newY);
    return this;
  }

  /**
   * returns the Vector 90 degrees counter-clockwise from this one
   *
   * @return the Vector 90 degrees counter-clockwise from this one
   */
  public Vector getPerpendicular() {
    return new Vector(-y(), x());
  }

  /**
   * rotates this Vector the given number of radians around the origin
   *
   * @param theta the number of radians to rotate this Vector around the origin
   * @return this Vector
   */
  public Vector rot(double theta) {
    double prevX = x();
    double prevY = y();
    setX(Math.cos(theta) * prevX - Math.sin(theta) * prevY);
    setY(Math.cos(theta) * prevY + Math.sin(theta) * prevX);
    return this;
  }

  /**
   * restricts this Point to a maximum length, setting it to that length it it it longer, then
   * returns itself
   *
   * @param maxLength the maximum length to be clamped to
   * @return this Vector
   */
  public Vector clampLength(double maxLength) {
    return clampLength(0, maxLength);
  }

  /**
   * restricts this Point to a maximum length, setting it to that length it it it longer, then
   * returns itself
   *
   * @param maxLength the maximum length to be clamped to
   * @return this Vector
   */
  public Vector clampLength(double minLength, double maxLength) {
    if (minLength > maxLength) throw new IllegalArgumentException();
    if (this.squaredMag() < minLength * minLength) {
      normalize();
      mult(minLength);
      return this;
    }
    if (this.squaredMag() > maxLength * maxLength) {
      normalize();
      mult(maxLength);
    }
    return this;
  }

  /**
   * clamps the value of a Point elementwise between the limits given by minimum and maximum Points
   * such that min.X() <= max.X() and min.Y() <= max.Y()
   *
   * @param min the minimum value for the dimensions of this Vector
   * @param max the maximum value for the dimentions of this Vector
   * @return this Vector
   */
  public Vector clamp(Vector min, Vector max) {
    if (min.dims() != max.dims()) {
      throw new IllegalArgumentException("arguments must have the same number of dimensions");
    }
    for (int i = 0; i < dims() && i < min.dims(); i++) {
      if (m_vals[i] < min.m_vals[i]) m_vals[i] = min.m_vals[i];
      if (m_vals[i] > max.m_vals[i]) m_vals[i] = max.m_vals[i];
    }
    return this;
  }

  /**
   * transforms this Point into the space of Point space this Point is also multiplied by the
   * magnitude of space in the process, so if you want to avoid this, normalize space first
   * effectively just a matrix transformation where space is iHat and jHat is space rotated 90
   * degreed counter-clockwise
   *
   * @param space the vector to the space of the given Vector
   * @return this Vector
   */
  public Vector toSpace(Vector space) {
    double oldX = x();
    double oldY = y();
    setX(oldX * space.x() + oldY * space.y());
    setY(oldY * space.x() - oldX * space.y());
    return this;
  }

  /**
   * returns whether this Vector is equivelent to another one in both number of dimensions and
   * values of dimensions
   *
   * @param p the Vector to compare to
   * @return whether the two Vectors are equal
   */
  public boolean equals(Vector p) {
    if (p.m_vals.length != this.m_vals.length) {
      return false;
    }
    for (int i = 0; i < m_vals.length; i++) {
      if (this.m_vals[i] != p.m_vals[i]) {
        return false;
      }
    }
    return true;
  }

  /**
   * gets the angle from the origin to this Vector in a counter-clockwise direction
   *
   * @return the angle from the origin to this Vector
   */
  public double angle() {
    return Math.atan2(y(), x());
  }

  /**
   * returns whether this Vector is within the given minimum and maximum bounds
   *
   * @param boundsMin the minumum values of the bounds
   * @param boundsMax the maximum values of the bounds
   * @return whether this Vector is within the given bounds
   */
  public boolean isWithinBounds(Vector boundsMin, Vector boundsMax) {
    double minX;
    double maxX;
    double minY;
    double maxY;

    if (boundsMin.x() < boundsMax.x()) {
      minX = boundsMin.x();
      maxX = boundsMax.x();
    } else {
      maxX = boundsMin.x();
      minX = boundsMax.x();
    }

    if (boundsMin.y() < boundsMax.y()) {
      minY = boundsMin.y();
      maxY = boundsMax.y();
    } else {
      maxY = boundsMin.y();
      minY = boundsMax.y();
    }

    return minX <= x() && x() <= maxX && minY <= y() && y() <= maxY;
  }
}