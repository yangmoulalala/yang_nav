#include <iostream>
#include <vector>
#include "nav_behaviors/zone_tracker.hpp"
using namespace nav_behaviors;
std::vector<Pose2> line(double first,double last){std::vector<Pose2> p;for(double x=first;x<=last+.0001;x+=.025)p.push_back({x,0,0});return p;}
int main(){Zone z;z.id="tunnel";z.depth=.35;z.width=.8;ZoneTracker tr;tr.setZones({z});tr.setPath(line(-2.,2.));const Pose2 r{.4,0,0};tr.compute({-.5,0,0});auto old=tr.compute(r);std::cout<<"full crossing path: phase="<<zonePhaseName(old.phase)<<" weight="<<old.weight<<" crossings="<<tr.crossingCount()<<"\n";tr.setPath(line(.4,2.));auto suffix=tr.compute(r);std::cout<<"replan suffix path: phase="<<zonePhaseName(suffix.phase)<<" weight="<<suffix.weight<<" crossings="<<tr.crossingCount()<<"\n";std::cout<<"rear coordinate="<<r.x-.31<<" far mouth="<<.175<<" (rear still inside body)\n";return old.weight>0&&suffix.weight==0?0:1;}