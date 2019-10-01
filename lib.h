#include <cmath>

struct MyDouble {
    static constexpr double eps = 1e-9;
    double val;
    MyDouble(): val(0) {}
    MyDouble(double x): val(x) {}
    MyDouble operator + (const MyDouble& other) const {
        return MyDouble(val + other.val);
    }
    MyDouble operator - (const MyDouble& other) const {
        return MyDouble(val - other.val);
    }
    MyDouble operator * (const MyDouble& other) const {
        return MyDouble(val * other.val);
    }
    MyDouble operator / (const MyDouble& other) const {
        return MyDouble(val / other.val);
    }
    MyDouble operator - () const {
        return MyDouble(-val);
    }
    int sign() {
        if (val < -eps) {
            return -1;
        }
        if (val > eps) {
            return 1;
        }
        return 0;
    }
};


std::ostream& operator << (std::ostream& out, const MyDouble& x) {
    out << x.val;
}

MyDouble sqrt(const MyDouble& x) {
    double tmp = x.val;
    return MyDouble(sqrt(x.val));
}

std::istream& operator >> (std::istream& in, MyDouble& x) {
    in >> x.val;
}

bool operator < (const MyDouble& x, const MyDouble& y) {
    return (x - y).sign() == -1;
}

bool operator == (const MyDouble& x, const MyDouble& y) {
    return (x - y).sign() == 0;
}

bool operator == (const MyDouble& x, const double& y) {
    return x == MyDouble(y);
}

bool operator > (const MyDouble& x, const MyDouble& y) {
    return y < x;
}


bool operator >= (const MyDouble& x, const MyDouble& y) {
    return !(x < y);
}

bool operator != (const MyDouble& x, const MyDouble& y) {
    return !(x == y);
}

bool operator <= (const MyDouble& x, const MyDouble& y) {
    return !(y < x);
}

MyDouble abs(const MyDouble& x) {
    return x >= 0 ? x : -x;
}

struct Vector{
    MyDouble x, y;
    Vector () {}
    Vector (MyDouble x, MyDouble y): x(x), y(y) {}
    Vector (double x, double y): x(MyDouble(x)), y(MyDouble(y)) {}
    Vector operator * (const double& k) {
        return Vector(x * MyDouble(k), y * MyDouble(k));
    }
    Vector operator * (const MyDouble& k) {
        return Vector(x * k, y * k);
    }
    Vector operator / (const double& k) {
        return Vector(x / MyDouble(k), y / MyDouble(k));
    }
    Vector operator / (const MyDouble& k) {
        return Vector(x / k, y / k);
    }
    MyDouble len() const {
        return sqrt(x * x + y * y);
    }
};

std::ostream& operator << (std::ostream& out, const Vector& u) {
    out << '(' << u.x << ' ' << u.y << ')';
}

std::istream& operator >> (std::istream& in, Vector& u) {
    in >> u.x >> u.y;
}

Vector operator + (const Vector& u, const Vector& v) {
    return Vector(u.x + v.x, u.y + v.y);
}

Vector operator - (const Vector& u, const Vector& v) {
    return Vector(u.x - v.x, u.y - v.y);
}

bool operator == (const Vector& u, const Vector& v) {
    return u.x == v.x && u.y == v.y;
}

bool operator != (const Vector& u, const Vector& v) {
    return !(u == v);
}

MyDouble dot_prod(const Vector& u, const Vector& v) {
    return u.x * v.x + u.y * v.y;
}

MyDouble cross_prod(const Vector& u, const Vector& v) {
    return u.x * v.y - u.y * v.x;
}

bool is_on_line(const Vector& p, const Vector& a, const Vector& b) { // is P on line AB
    return cross_prod(a - p, b - p) == 0;
}

bool is_on_segment(const Vector& p, const Vector& a, const Vector& b) { // is P on segment AB
    return is_on_line(p, a, b) && dot_prod(p - a, p - b).sign() <= 0;
}

Vector find_lines_intersection(const Vector& a1, const Vector& b1, const Vector& a2, const Vector& b2) { // intersection of lines A1B1 and A2B2
    return a1 + (b1 - a1) * (cross_prod(a2 - a1, b2 - a2) / cross_prod(b1 - a1, b2 - a2));
}

bool does_segment_intersect_line (const Vector& s1, const Vector& s2, const Vector& a, const Vector& b) { // does segment S1S2 intersect line AB
    return cross_prod(s1 - a, b - a).sign() * cross_prod(s2 - a, b - a).sign() <= 0;
}

bool are_segments_intersecting (const Vector& a1, const Vector& b1, const Vector& a2, const Vector& b2) { // does segment S1S2 intersect line AB
    if (a1 == b1) {
        if (a2 == b2) {
            return a1 == a2;
        }
        return is_on_segment(a1, a2, b2);
    }
    if (a2 == b2) {
        return is_on_segment(a2, a1, b1);
    }
    if (is_on_line(a1, a2, b2) && is_on_line(b1, a2, b2)) {
        return is_on_segment(a1, a2, b2) || is_on_segment(b1, a2, b2) || is_on_segment(a2, a1, b1) || is_on_segment(b2, a1, b1);
    }
    return does_segment_intersect_line(a1, b1, a2, b2) && does_segment_intersect_line(a2, b2, a1, b1);
}
bool does_ray_intersect_line(const Vector& s, const Vector& x, const Vector& a, const Vector& b) { // does ray SX intersect line AB
    return does_segment_intersect_line(s, x, a, b) || abs(cross_prod(a - b, s - a)) > abs(cross_prod(a - b, x - a));
}

bool does_ray_intersect_segment(const Vector& s, const Vector& x, const Vector& a, const Vector& b) { // does ray SX intersect segment AB
    return does_ray_intersect_line(s, x, a, b) && does_segment_intersect_line(a, b, s, x);
}

double unoriented_angle(const Vector& u, const Vector& v) {
    double res = abs(dot_prod(u, v) / (u.len() * v.len())).val;
    if (res > 1) {
        res = 1;
    }
    if (res < -1) {
        res = -1;
    }
    return acos(res);
}

Vector rotated(const Vector& v, double angle) {
    return Vector(v.x * cos(angle) - v.y *sin(angle), v.y * cos(angle) + v.x * sin(angle));
}

MyDouble dist_between_point_and_line(const Vector& p, const Vector& a, const Vector& b) {
    return abs(cross_prod(p - a, b - a) / (b - a).len());
}

MyDouble dist_between_point_and_segment(const Vector& p, const Vector& a, const Vector& b) {
    if (dot_prod(p - a, b - a) <= 0) {
        return (p - a).len();
    }
    if (dot_prod(p - b, a - b) <= 0) {
        return (p - b).len();
    }
    return dist_between_point_and_line(p, a, b);
}

MyDouble dist_between_unintersecting_segments(const Vector& s1, const Vector& f1, const Vector& s2, const Vector& f2) {
    MyDouble res;
    res = dist_between_point_and_segment(s1, s2, f2);
    if (res > dist_between_point_and_segment(f1, s2, f2)) {
        res = dist_between_point_and_segment(f1, s2, f2);
    }

    if (res > dist_between_point_and_segment(s2, s1, f1)) {
        res = dist_between_point_and_segment(s2, s1, f1);
    }
    if (res > dist_between_point_and_segment(f2, s1, f1)) {
        res = dist_between_point_and_segment(f2, s1, f1);
    }
    return res;
}

MyDouble dist_between_segments(const Vector& s1, const Vector& f1, const Vector& s2, const Vector& f2) {
    if (are_segments_intersecting(s1, f1, s2, f2)) {
        return 0;
    }
    return dist_between_unintersecting_segments(s1, f1, s2, f2);
}

