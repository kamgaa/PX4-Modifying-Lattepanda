#include "palletrone_task_flight/joint_math.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
void check(bool condition) {if(!condition) throw std::runtime_error("joint math test failed");}
template<class F> void rejects(F f) {bool failed=false;try{f();}catch(const std::invalid_argument&){failed=true;}check(failed);}
int main() {
  using namespace task_flight;
  Calibration c; c.validate(); rejects([&]{c.raw(0);});
  c.confirmed=true;
  check(c.raw(0)==2048);
  check(c.raw(pi/6)==2389);
  check(std::abs(c.angle(c.raw(pi/6))-pi/6)<2*pi/4096);
  c.direction=-1;check(c.raw(pi/6)==1707);
  rejects([&]{c.raw(1.0);});rejects([&]{c.raw(std::numeric_limits<double>::quiet_NaN());});
  rejects([&]{c.angle(-1);});rejects([&]{c.angle(5000);});
  c.direction=0;rejects([&]{c.validate();});
  check(legacy_tick(0)==2048);check(legacy_tick(-3.14)==0);check(legacy_tick(3.14)==4096);
  auto a=legacy_commands({0,0.1f,-0.1f,0.2f,999.0f});
  auto b=legacy_commands({0,0.1f,-0.1f,0.2f,-999.0f});check(a==b);
  auto feedback=legacy_feedback({2048,2048,2048,2048});check(feedback.size()==5);
  check(feedback[0]==0);check(feedback[4]==legacy_angle(0));
  rejects([&]{legacy_commands({0,0,0,0});});
  // Compare the complete single-turn feedback domain with the actual legacy expression.
  for(uint32_t raw=0;raw<4096;++raw) {
    float original=(static_cast<float>(static_cast<uint16_t>(raw))/4096.0f)*(2.0f*3.14)-3.14;
    check(legacy_angle(raw)==original);
  }
  for(int k=-700;k<=700;++k) {
    float radians=k/1000.0f;
    uint16_t original=((3.14+radians)/(2*3.14))*4096;
    check(legacy_tick(radians)==original);
  }
  std::cout<<"J1 calibration and legacy ID1..4 contract: PASS\n";
}
