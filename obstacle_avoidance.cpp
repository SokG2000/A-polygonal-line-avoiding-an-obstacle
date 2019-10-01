#include <iostream>
#include <vector>
#include "lib.h"

using std::cin;
using std::cout;

void make_nearer_if_need(const Vector& center, Vector& point, MyDouble eps) {
    // if distance between center and point more than eps, move point to distance eps from center
    Vector dir = point - center;
    if (dir.len() > eps) {
        point = center + dir * (eps / dir.len());
    }
}

std::vector<Vector> segment_avoidance(const Vector& start1, const Vector& finish1, const Vector& start2, const Vector& finish2, const Vector& s1, const Vector& s2, MyDouble eps, double angle) {
    // start1, finish1 - first intersection, start2, finish2 - last intersection, s1s2 - added segment of polyline
    // makes path from start1 to finish2
    std::vector<Vector> res;
    Vector s1s2 = s2 - s1;
    Vector p1, p2, p3;
    p1 = s1 + rotated(s1s2, angle);
    p2 = s2 + s1s2 * (eps / s1s2.len());
    p3 = s1 + rotated(s1s2, -angle);
    if (cross_prod(s1s2, p1 - s1).sign() != cross_prod(s1s2, start1 - s1).sign()) {
        std::swap(p1, p3);
    } // now p1 and start1 in one half-plane for s1s2
    make_nearer_if_need(s2, p1, eps);
    make_nearer_if_need(s2, p3, eps);
    if (cross_prod(s1s2, p1 - s1).sign() != cross_prod(s1s2, finish2 - s1).sign()) {
        // start1 and finish2 are in different half-planes for s1s2
        if (are_segments_intersecting(start1, finish1, s1, p1)) {
            res.push_back(find_lines_intersection(start1, finish1, s1, p1));
            res.push_back(p1);
        } else {
            res.push_back(find_lines_intersection(start1, finish1, p1, p2));
        }
        res.push_back(p2);
        if (are_segments_intersecting(start2, finish2, s1, p3)) {
            res.push_back(p3);
            res.push_back(find_lines_intersection(start2, finish2, s1, p3));
        } else {
            res.push_back(find_lines_intersection(start2, finish2, p3, p2));
        }
    } else {
        if (are_segments_intersecting(start1, finish1, s1, p1)) {
            res.push_back(find_lines_intersection(start1, finish1, s1, p1));
        } else {
            res.push_back(find_lines_intersection(start1, finish1, p1, p2));
        }
        if (are_segments_intersecting(start2, finish2, s1, p1)) {
            res.push_back(find_lines_intersection(start2, finish2, s1, p1));
        } else {
            res.push_back(find_lines_intersection(start2, finish2, p1, p2));
        }
    }
/*
    cout << "new_way: ";
    for (int i = 0; i < res.size(); i++) {
        cout << res[i] << ' ';
    }
    cout << '\n';
*/
    return res;
}

MyDouble find_eps(const std::vector <Vector> &polyline, const Vector& start, const Vector& finish, int n) {
    MyDouble eps = 1;
    for (int i = 0; i < n - 3; i++) {
        if (dist_between_unintersecting_segments(polyline[i], polyline[i + 1], polyline[n - 2], polyline[n - 1]) < eps) {
            eps = dist_between_unintersecting_segments(polyline[i], polyline[i + 1], polyline[n - 2], polyline[n - 1]);
        }
    }
    if (dist_between_point_and_segment(start, polyline[n - 2], polyline[n - 1]) < eps) {
        eps = dist_between_point_and_segment(start, polyline[n - 2], polyline[n - 1]);
    }
    if (dist_between_point_and_segment(finish, polyline[n - 2], polyline[n - 1]) < eps) {
        eps = dist_between_point_and_segment(finish, polyline[n - 2], polyline[n - 1]);
    }
    return eps;
}

void add_segment(const std::vector<Vector>& obstacle, std::vector<Vector>& path, const Vector& start, const Vector& finish, int last_start) {
    double last_angle;
    if (last_start == 0) {
        last_angle = 1;
    } else {
        last_angle = unoriented_angle(obstacle[last_start] - obstacle[last_start + 1], obstacle[last_start] - obstacle[last_start - 1]);
    }
    MyDouble eps = find_eps(obstacle, start, finish, last_start + 2);
    int first_intersection = -1;
    int last_intersection = -1;
    for (int i = 0; i < path.size() - 1; i++) {
        if (are_segments_intersecting(path[i], path[i + 1], obstacle[last_start], obstacle[last_start + 1])) {
            if (first_intersection == -1) {
                first_intersection = i;
            }
            last_intersection = i;
        }
    }
    if (first_intersection != -1) {
        std::vector <Vector> new_way = segment_avoidance(path[first_intersection], path[first_intersection + 1], path[last_intersection], path[last_intersection + 1], obstacle[last_start], obstacle[last_start + 1], eps, last_angle);
        std::vector <Vector> new_path;
        new_path.reserve(path.size() + new_way.size() - (last_intersection - first_intersection));
        for (int i = 0; i <= first_intersection; i++) {
            new_path.push_back(path[i]);
        }
        for (int i = 0; i < new_way.size(); i++) {
            new_path.push_back(new_way[i]);
        }
        for (int i = last_intersection + 1; i < path.size(); i++) {
            new_path.push_back(path[i]);
        }
        std::swap(new_path, path);
    }
}

int main() {
    int n;
    double x, y;
    Vector a, b;
    cout << "Give the number of vertexes of a polygonal line L.\n";
    cin >> n;
    cout << "Give coordinates of vertexes of L: for each point give two real numbers, separated by space. Vertexes must by separated by space or by enter.\n";
    std::vector <Vector> polyline(n);
    for (int i = 0; i < n; i++) {
        cin >> polyline[i];
    }
    cout << "Give coordinates of two points A and B in the same format\n";
    cin >> a >> b;
    std::vector <Vector> path;
    path.push_back(a);
    if (is_on_line(polyline[0], a, b)) {
        path.push_back(a + rotated(b - a, 1));
    }
    path.push_back(b);
    for (int i = 0; i < polyline.size() - 1; i++) {
        add_segment(polyline, path, a, b, i);
    }
    cout << "A polygonal line that connect point A and B and does not cross polygonal line L has vertexes ";
    for (int i = 0; i < path.size() - 1; i++) {
        cout << path[i] << ", ";
    }
    cout << path[path.size() - 1] << ".\n";
    return 0;
}
