#include "controller.h"
#include <cassert>
#include <iostream>
using namespace icd;
int main() {
 Controller c;
 c.move=[](int,int){}; c.scanMove=[](int,int,int){};
 c.hold=[](){}; c.light=[](bool){return true;};
 c.sample(2048,1024,0);
 c.receive(packet(EF01,0,0),"p",0);
 unsigned seq=0;
 const calibration::Point points[]={{45,8,10},{315,20,-10}};
 for(int mode=0;mode<2;++mode) {
  if(mode) c.correction=calibration::Map(points,2);
  for(int p: {1700,1800,1950,2100}) {
   int t=123;
   auto target=c.correction.apply(ControlFsm::panTicksFromDecidegrees(p),ControlFsm::tiltTicksFromDecidegrees(t));
   c.sample(target.pan,target.tilt,0);
   auto r=c.receive(packet(EF02,++seq,0),"p",0);
   assert(std::abs(signedWord(r,8)-t)<=1);
   assert(std::abs(signedWord(r,10)-p)<=1);
   auto raw=c.receive(packet(EF08,++seq,0),"p",0);
   assert(signedWord(raw,8)==target.tilt && signedWord(raw,10)==target.pan);
  }
 }
 c.ready=false;
 auto r=c.receive(packet(EF02,++seq,0),"p",0);
 assert(signedWord(r,8)==UNKNOWN_ANGLE && signedWord(r,10)==UNKNOWN_ANGLE);
 std::cout<<"ICD offset-removal tests passed\n";
}
