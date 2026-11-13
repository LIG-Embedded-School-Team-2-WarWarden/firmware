#include "controller.h"
#include <cassert>
#include <iostream>
using namespace icd;
struct Fixture {
 Controller c; int sends=0, holds=0;
 Fixture() {
  c.move=[](int,int){};
  c.scanMove=[this](int,int,int speed){assert(speed==2000);++sends;};
  c.hold=[this](){++holds;};c.light=[](bool){return true;};
  c.sample(2048,1024,0);c.receive(packet(EF01,0,0),"p",0);
 }
 void sample(long long now) { c.sample(2048,1024,now);c.tick(now); }
};
int main() {
 for(int mode:{EF05,EF06,EF07}) {
  Fixture f;
  assert(word(f.c.receive(packet(mode,1,0),"p",0),6)==ACCEPTED);
  assert(f.sends==1 && f.c.scanPath.size()==47);
  for(size_t i=1;i<f.c.scanPath.size();++i)
   assert(f.c.scanPath[i].pan!=f.c.scanPath[i-1].pan);
  f.sample(199);assert(f.sends==1);
  f.sample(200);assert(f.sends==2); // actual position still center, no arrival
  for(int k=2;k<=46;++k) f.sample(k*200);
  assert(f.sends==47 && !f.c.returning && f.c.laser);
  f.sample(9399);assert(!f.c.returning);
  f.sample(9400);assert(f.c.returning && !f.c.laser && f.sends==48);
  assert(f.c.demo); // Return still waits for physical arrival/stability.
  f.c.sample(f.c.returnPan,f.c.returnTilt,9450);f.c.tick(9450);
  f.c.sample(f.c.returnPan,f.c.returnTilt,9600);f.c.tick(9600);
  assert(!f.c.demo && !f.c.moving && !f.c.fault);
 }
 {
  Fixture f;f.c.receive(packet(EF05,1,0),"p",0);
  f.c.receive(packet(EF04,2,0,{0}),"p",100);
  assert(f.c.returning && !f.c.laser);
  int count=f.sends;f.sample(200);assert(f.sends==count);
 }
 {
  Fixture f;f.c.receive(packet(EF05,1,0),"p",0);
  f.c.tick(501);assert(f.c.fault && !f.c.laser && f.holds==1);
 }
 {
  Fixture f;f.c.receive(packet(EF05,1,0),"p",0);
  f.sample(450);assert(f.sends==2); // Late tick never emits catch-up burst.
  f.sample(649);assert(f.sends==2);
  f.sample(650);assert(f.sends==3);
 }
 std::cout<<"Timed scan tests passed\n";
}
