#!/usr/bin/env python3
"""Run the actual loop1 window/reset logic with deterministic audio/time fakes."""
from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parents[1]
sketch = (root/'Arduino/VoorburgSoundsKit/VoorburgSoundsKit.ino').read_text()
start=sketch.index('void loop1() {'); end=sketch.index('static void publishClockAnchor()',start)
worker=sketch[start:end]
start=sketch.index('static uint8_t toUQ7_1('); end=sketch.index('static void logPreparedMeasurement(',start)
quantizers=sketch[start:end]
stub=r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <atomic>
#include "MeasurementHandoff.h"
using namespace MeasurementHandoff;
LatestMailbox measurementMailbox; UtcClock measurementClock;
FlashPause flashPause; CoreHeartbeat audioHeartbeat;
std::atomic<uint32_t> audioBootstrapReady{0},audioEvents{0},audioSamplesReported{0},audioLevelsReported{0},audioLevelReported{0};
const uint64_t MEASUREMENT_PERIOD_US=300000000ULL;
#define MIC_SAMPLE_STALL_MS 15000UL
uint64_t nowUs=1000000; uint64_t time_us_64(){return nowUs;}
uint32_t millis(){return static_cast<uint32_t>(nowUs/1000);}
enum AudioEvent:uint32_t{AUDIO_STARTED=1,AUDIO_START_FAILED=2,AUDIO_STALLED=4,AUDIO_EMPTY_WINDOW=8,AUDIO_PAUSE_FAILED=16,AUDIO_RESUME_FAILED=32};
enum AudioOperation:uint32_t{AUDIO_START=1,AUDIO_READ=2,AUDIO_STOP=3,AUDIO_PACK=4,AUDIO_FLASH_PAUSED=5,AUDIO_HANDOFF=6};
void markAudioOperation(uint32_t){}
struct Stats {uint32_t n=0; uint32_t N() const{return n;} double Mean() const{return 40;}
 double Min()const{return 30;} double Max()const{return 60;} double StdDev(bool)const{return 4;}};
struct SLM {Stats statsDB,statsRMS; bool active=false; uint32_t samples=0,starts=0,stops=0;
 double latestLevelRelative=40;
 bool start(){active=true;++starts;return true;} bool pause(){active=false;return true;}
 bool stop(){active=false;samples=0;++stops;return true;}
 void update(){assert(active);++samples;statsDB.n+=4;statsRMS.n+=4;}
 uint32_t samplesRead(){return samples;} void resetStatistics(){statsDB.n=statsRMS.n=0;}} slm;
'''
checks=r'''
int main(){
 loop1();assert(!slm.active&&audioHeartbeat.started());
 audioBootstrapReady.store(1);assert(measurementClock.tryUpdate(nowUs,1791532800u));
 loop1();assert(slm.starts==1&&slm.samples>0);
 nowUs=301000000;loop1();
 assert(slm.stops==1&&slm.starts==2&&slm.active&&slm.samples==0&&slm.statsDB.N()==0);
 // Leave mailbox unread: the next window replaces it without network work.
 nowUs+=1000;loop1(); nowUs=601000000;loop1();
 Measurement out;for(unsigned tries=0;tries<1000&&!measurementMailbox.tryTakeLatest(out,nowUs);++tries)loop1();
 assert(out.sequence==2&&out.completedUs==601000000&&out.expiresUs==901000000);
 assert(out.windowEndUtc()==1791533400u&&slm.stops==2&&slm.starts==3&&slm.samples==0);
 nowUs+=1000;loop1();assert(slm.statsDB.N()>1);
 assert(flashPause.request(1));loop1();
 assert(flashPause.acknowledged(1)&&!slm.active);
 // Boundary during flash pause still resets counters; restart waits for release.
 nowUs=901000000;loop1();assert(slm.samples==0&&slm.statsDB.N()==0&&!slm.active);
 assert(slm.starts==3&&slm.stops==3);
 assert(flashPause.release(1));nowUs+=1000;loop1();assert(slm.active&&slm.starts==4);
 assert(audioHeartbeat.fresh(nowUs,3000000));
}
'''
with tempfile.TemporaryDirectory(prefix='voorburg-audio-worker-') as directory:
 source=Path(directory)/'check.cpp'; binary=Path(directory)/'check'
 source.write_text(stub+quantizers+worker+checks)
 subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-I'+str(root/'Arduino/VoorburgSoundsKit'),str(source),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print('PASS: actual audio worker resets counters/statistics after each payload, replaces old snapshots, and resumes after flash pause')
