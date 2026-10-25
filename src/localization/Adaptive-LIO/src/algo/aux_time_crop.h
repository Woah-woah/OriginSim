#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
namespace zjloc {
template<class Point> void SortFinitePoints(std::vector<Point>& points) {
    points.erase(std::remove_if(points.begin(),points.end(),[](const Point& p){
        return !std::isfinite(p.timestamp)||!p.raw_point.allFinite();
    }),points.end());
    std::stable_sort(points.begin(),points.end(),[](const Point& a,const Point& b){return a.timestamp<b.timestamp;});
}
// Half-open [begin,end): future points remain in the packet, expired points
// are consumed. If the main windows overlap, the earlier window owns a point.
template<class Point> void CropAuxPoints(const std::vector<Point>& points,size_t& cursor,
                                         double begin,double end,double offset,std::vector<Point>& out) {
    while(cursor<points.size()){
        const double t=points[cursor].timestamp+offset;
        if(t>=end)break;
        if(t>=begin){auto p=points[cursor];p.timestamp=t;out.push_back(std::move(p));}
        ++cursor;
    }
}
} // namespace zjloc
