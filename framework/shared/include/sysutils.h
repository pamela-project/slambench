/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef SYSUTILS_H_
#define SYSUTILS_H_

struct MemInfos {
	long long totalVirtualMem;
	long long virtualMemUsed;
	long long totalPhysMem;
	long long physMemUsed;
};

#include "sys/types.h"
#ifdef __APPLE__
inline MemInfos getMemInfos() {
  MemInfos res = {0,0,0,0};
  return res;
}
#else

#include "sys/sysinfo.h"


inline MemInfos getMemInfos() {
  
  MemInfos res;

  struct sysinfo memInfo;
  sysinfo (&memInfo);
  
 
  long long totalVirtualMem = memInfo.totalram;
  //Add other values in next statement to avoid int overflow on right hand side...
  totalVirtualMem += memInfo.totalswap;
  totalVirtualMem *= memInfo.mem_unit;
  res.totalVirtualMem = totalVirtualMem;

  long long virtualMemUsed = memInfo.totalram - memInfo.freeram;
  //Add other values in next statement to avoid int overflow on right hand side...
  virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
  virtualMemUsed *= memInfo.mem_unit;
  res.virtualMemUsed = virtualMemUsed;

  long long totalPhysMem = memInfo.totalram;
  //Multiply in next statement to avoid int overflow on right hand side...
  totalPhysMem *= memInfo.mem_unit;
  res.totalPhysMem = totalPhysMem;
  
  long long physMemUsed = memInfo.totalram - memInfo.freeram;
  //Multiply in next statement to avoid int overflow on right hand side...
  physMemUsed *= memInfo.mem_unit;
  res.physMemUsed = res.physMemUsed;
    
  return res;
}

#endif

inline int parseLine(char* line){
      int i = strlen(line);
      while (*line < '0' || *line > '9') line++;
      line[i-3] = '\0';
      i = atoi(line);
      return i;
  }


inline  int getValueVmSize(){ //Note: this value is in KB!
      FILE* file = fopen("/proc/self/status", "r");
      int result = -1;
      char line[128];


      while (fgets(line, 128, file) != NULL){
          if (strncmp(line, "VmSize:", 7) == 0){
              result = parseLine(line);
              break;
          }
      }
      fclose(file);
      return result;
  }

inline   int getValueVmRSS(){ //Note: this value is in KB!
        FILE* file = fopen("/proc/self/status", "r");
        int result = -1;
        char line[128];


        while (fgets(line, 128, file) != NULL){
            if (strncmp(line, "VmRSS:", 6) == 0){
                result = parseLine(line);
                break;
            }
        }
        fclose(file);
        return result;
    }




#endif /* SYSUTILS_H_ */
