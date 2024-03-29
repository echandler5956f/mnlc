#include "math.h"

using namespace DStarLite;

/**
 * @var double INF
 */
const double Math::INF = DBL_MAX;

/**
 * @var double SMALL
 */
const double Math::SMALL = 0.000001;

/**
 * @var double PI
 */
const double Math::PI = 3.14159265;

/**
 * @var double EUL
 */
const double Math::EUL = 2.71828182845904;

/**
 * @var double SQRT2
 */
const double Math::SQRT2 = 1.41421356237309504880;

/**
 * Converts degrees to radians.
 *
 * @param double degrees
 * @return double radians
 */
double Math::deg2rad(double degrees)
{
	return degrees * Math::PI / 180.0;
}

/**
 * Converts degrees to a signed degree (-180 < degrees <= 180)
 *
 * @param double degrees
 * @return double degrees
 */
double Math::deg2signed(double degrees)
{
	degrees = fmod(degrees, 360.0);

	return (degrees > 180.0) ? -1.0 * fmod(degrees, 180.0) : degrees;
}

/**
 * Determines if two doubles are equal based on a precision.
 *
 * @param double first double
 * @param double second double
 * @param double [optional] precision
 * @return bool
 */
bool Math::equals(double a, double b, double precision)
{
	if (a == Math::INF && b == Math::INF)
		return true;

	return (fabs(a - b) < precision);
}

/**
 * Determines if a double is greater than compared to another double
 * based on a precision.
 *
 * @param double first double
 * @param double second double
 * @param double [optional] precision
 * @return bool
 */
bool Math::greater(double a, double b, double precision)
{
	if (a == Math::INF && b == Math::INF)
		return false;

	return a - precision > b;
}

/**
 * Determines if a double is less than compared to another double
 * based on a precision.
 *
 * @param double first double
 * @param double second double
 * @param double [optional] precision
 * @return bool
 */
bool Math::less(double a, double b, double precision)
{
	if (a == Math::INF && b == Math::INF)
		return false;

	return a + precision < b;
}

/**
 * Determines the minimum of three ints for convenience.
 *
 * @param int first int
 * @param int second int
 * @param int third int
 * @return int minimum
 */
int Math::min3(int a, int b, int c)
{
	if (a <= b && a <= c)
		return a;
	else if (b <= c)
		return b;
	return c;
}

/**
 * Converts radians to degrees.
 *
 * @param double radians
 * @return double degrees
 */
double Math::rad2deg(double radians)
{
	return radians * 180.0 / Math::PI;
}

/**
 * Converts radians to a signed radians value (-PI < degrees <= PI)
 *
 * @param double radians
 * @return double radians
 */
double Math::rad2signed(double radians)
{
	radians = fmod(radians, (2.0 * Math::PI));

	return (radians > Math::PI) ? -1.0 * fmod(radians, Math::PI) : radians;
}