// Finite communication-only probe. BRAKE mode, all command values zero.
// Run via probe_hardware_comms.py for process-ownership preflight and JSON output.
// No FOC/calibration mode exists in this utility; never use the old IOSDK ctor.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "serialPort/SerialPort.h"
#include "crc/crc_ccitt.h"
int main(int argc, char** argv) {
  const std::string path = argc > 1 ? argv[1] : "motor-results.json";
  const int samples = argc > 2 ? std::stoi(argv[2]) : 20;
  if (samples < 1 || samples > 500) return 3;
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != 0) return 4;
  const char* legs[] = {"FR", "FL", "RR", "RL"};
  const char* ports[] = {"/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS7", "/dev/ttyS8"};
  std::ofstream out(path);
  out << std::setprecision(10) << "{\"mode\":0,\"tau\":0,\"kp\":0,\"kd\":0,\"baud\":4000000,\"samples_per_motor\":" << samples << ",\"motors\":[";
  bool first=true; int total_ok=0, total_faults=0;
  for (int leg=0; leg<4; ++leg) {
    SerialPort serial(ports[leg], 16, 4000000, 2000, BlockYN::NO,
                      bytesize_t::eightbits, parity_t::parity_none,
                      stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none);
    for (int id=0; id<3; ++id) {
      MotorCmd cmd; cmd.motorType=MotorType::GO_M8010_6; cmd.hex_len=17;
      std::memset(cmd.get_motor_send_data(), 0, cmd.hex_len);
      cmd.id=id; cmd.mode=0; cmd.tau=cmd.dq=cmd.q=cmd.kp=cmd.kd=0.0f;
      MotorData data; data.motorType=MotorType::GO_M8010_6; data.hex_len=16;
      std::memset(data.get_motor_recv_data(), 0, data.hex_len);
      int valid=0, sdk_ok=0, crc_fail=0, id_fail=0, mode_fail=0, motor_errors=0;
      double q_min=1e100,q_max=-1e100,dq_abs_max=0,tau_abs_max=0; int temp=0,last_error=-1;
      std::vector<double> times;
      for(int n=0;n<samples;++n) {
        data.correct=false;
        const auto start=std::chrono::steady_clock::now();
        cmd.modify_data(&cmd);
        const bool received=serial.sendRecv(cmd.get_motor_send_data(),data.get_motor_recv_data(),cmd.hex_len);
        const bool ok=received && data.extract_data(&data);
        times.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count());
        const auto* raw=data.get_motor_recv_data();
        const bool crc_ok=raw[0]==0xfd && raw[1]==0xee && crc_ccitt(0,raw,14)==(raw[14]|(raw[15]<<8));
        sdk_ok += ok && data.correct;
        if(ok && data.correct && !crc_ok) ++crc_fail;
        if(ok && data.correct && data.motor_id!=id) ++id_fail;
        if(ok && data.correct && data.mode!=0) ++mode_fail;
        if(ok && data.correct && crc_ok && data.motor_id==id && data.mode==0 && std::isfinite(data.q) && std::isfinite(data.dq) && std::isfinite(data.tau)) {
          ++valid; q_min=std::min(q_min,double(data.q)); q_max=std::max(q_max,double(data.q));
          dq_abs_max=std::max(dq_abs_max,std::abs(double(data.dq)));
          tau_abs_max=std::max(tau_abs_max,std::abs(double(data.tau)));
          temp=data.temp; last_error=data.merror; motor_errors+=data.merror!=0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      std::sort(times.begin(),times.end()); double sum=0;for(double t:times)sum+=t;
      if(!first)out<<','; first=false;
      out<<"{\"leg\":\""<<legs[leg]<<"\",\"port\":\""<<ports[leg]<<"\",\"id\":"<<id<<",\"valid\":"<<valid<<",\"sdk_valid\":"<<sdk_ok<<",\"crc_fail\":"<<crc_fail<<",\"id_fail\":"<<id_fail<<",\"mode_fail\":"<<mode_fail<<",\"motor_error_samples\":"<<motor_errors<<",\"last_error\":"<<last_error<<",\"temperature_c\":"<<temp<<",\"q_rotor_min_rad\":"<<(valid?q_min:0)<<",\"q_rotor_max_rad\":"<<(valid?q_max:0)<<",\"dq_rotor_abs_max_rad_s\":"<<dq_abs_max<<",\"tau_rotor_abs_max_nm\":"<<tau_abs_max<<",\"roundtrip_mean_us\":"<<sum/samples<<",\"roundtrip_p99_us\":"<<times[std::min(samples-1,int(std::ceil(.99*samples))-1)]<<",\"roundtrip_max_us\":"<<times.back()<<'}';
      total_ok+=valid; total_faults+=motor_errors;
      std::cerr<<legs[leg]<<" id="<<id<<" valid="<<valid<<"/"<<samples<<" sdk="<<sdk_ok<<" error="<<last_error<<" mean_us="<<sum/samples<<'\n';
    }
  }
  out<<"],\"total_valid\":"<<total_ok<<",\"total_attempts\":"<<samples*12<<"}\n";
  return total_ok==samples*12 && total_faults==0?0:2;
}
