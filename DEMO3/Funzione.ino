double degToRad(double degrees) {
  return degrees*pi / 180;
}
double radToDeg(double radians) {
  return radians * 180 / pi;
}
double Quadratic(double a, double b, double c, double plusOrMinus) {
  plusOrMinus = plusOrMinus / sqrt(pow(plusOrMinus, 2));
  return (-b + plusOrMinus*sqrt(pow(b, 2) - 4 * a*c)) / (2 * a);
}
