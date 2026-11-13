#include "jog_core.hpp"
#include <cassert>
#include <iostream>
#include <limits>
int main() {
  using namespace jog;
  Gate g;
  Input neutral; neutral.fresh=true;
  Input y=neutral; y.y=true;
  Input a=neutral; a.a=true;
  assert(g.update(y,0)==0 && g.update(y,1)==0); // no startup-held key
  g.update(neutral,2);
  assert(g.update(y,3)==0 && g.update(y,3.3)==1);
  assert(g.update(neutral,3.31)==0); // immediate release
  Input modified=y; modified.other=true;
  g.update(modified,4);
  assert(g.update(y,5)==0); // modifier released before face key
  g.update(neutral,6);g.update(a,7);
  assert(g.update(a,7.3)==-1);
  Input both=y; both.a=true;
  assert(g.update(both,7.31)==0 && g.update(y,8)==0);
  g.update(neutral,9);g.update(y,10);
  y.fresh=false; assert(g.update(y,10.3)==0);
  y.fresh=true; assert(g.update(y,11)==0); // no reconnect-held restart
  g.update(neutral,12);g.update(y,13);
  Input walking=y;walking.axes=true;
  assert(g.update(walking,13.3)==0 && g.update(y,14)==0);
  g.update(neutral,15);g.update(y,16);assert(g.update(y,16.3)==1);
  assert(g.update(a,16.31)==0 && g.update(a,17)==0); // reverse needs release
  g.update(neutral,18);Input close=neutral;close.close=true;
  assert(g.update(close,19)==0 && g.update(close,19.3)==2);
  Input open=neutral;open.open=true;
  assert(g.update(open,19.31)==0 && g.update(open,20)==0);
  g.update(neutral,21);g.update(open,22);assert(g.update(open,22.3)==-2);
  Input conflict=open;conflict.close=true;assert(g.update(conflict,22.31)==0);
  g.update(neutral,23);g.update(close,24);close.other=true;assert(g.update(close,24.3)==0);
  close.other=false;assert(g.update(close,25)==0); // modifiers cannot leave a held L3 active
  Pose p=retracted; double speed=0;
  double initial_left=wrist_y(p,0),initial_right=wrist_y(p,1);
  for(int n=0;n<1600;++n) {
    Pose before=p; advance(p,1,.01,speed);
    assert(within_envelope(p));
    for(unsigned i=0;i<8;++i) assert(std::abs(p[i]-before[i])<=max_speed*.01+1e-12);
  }
  for(unsigned i: {0u,3u,4u,7u}) assert(std::abs(p[i]-extended[i])<1e-8);
  assert(std::abs(wrist_y(p,0)-initial_left)<1e-9 && std::abs(wrist_y(p,1)-initial_right)<1e-9);
  speed=0;
  for(int n=0;n<1600;++n) advance(p,-1,.01,speed);
  for(unsigned i=0;i<8;++i) assert(std::abs(p[i]-retracted[i])<1e-8);
  Pose before=p;speed=0;assert(advance(p,2,.01,speed));assert(p==before); // do not close beside torso
  p=extended;speed=0;
  for(int n=0;n<1600;++n) {
    before=p;advance(p,2,.01,speed);
    assert(wrist_gap(p)>=min_wrist_gap && wrist_gap(p)<=wrist_gap(before)+1e-12);
    for(unsigned i=0;i<8;++i) assert(std::abs(p[i]-before[i])<=wrist_speed*.01+1e-12);
    for(unsigned i: {0u,2u,3u,4u,6u,7u}) assert(p[i]==extended[i]);
  }
  assert(wrist_gap(p)<min_wrist_gap+.001 && within_envelope(p));
  double closed_left=wrist_y(p,0),closed_right=wrist_y(p,1);speed=0;
  for(int n=0;n<1600;++n) advance(p,-1,.01,speed);
  assert(p[0]<=forward_pitch+1e-8 && p[4]<=forward_pitch+1e-8); // protect torso while clamped
  assert(std::abs(wrist_y(p,0)-closed_left)<1e-9 && std::abs(wrist_y(p,1)-closed_right)<1e-9);
  speed=0;
  for(int n=0;n<1600;++n) {before=p;advance(p,-2,.01,speed);assert(wrist_gap(p)>=wrist_gap(before)-1e-12);}
  assert(std::abs(p[1]-extended[1])<1e-8 && std::abs(p[5]-extended[5])<1e-8);
  before=p;speed=0;for(int n=0;n<1600;++n) advance(p,-1,.01,speed);
  assert(std::abs(p[0]-retracted[0])<1e-8 && std::abs(p[4]-retracted[4])<1e-8);
  bool failed=false;try{advance(p,1,.1,speed);}catch(...){failed=true;}assert(failed);
  p[0]=std::numeric_limits<double>::quiet_NaN();assert(!within_envelope(p));
  p=retracted;p[0]-=.3;assert(!tracking(p,retracted));
  std::cout<<"PASS: four jog keys, release, combos, startup/reconnect, stale input, limits, wrist gap, torso clearance, aperture preservation, speed, tracking, deadlines\n";
}
