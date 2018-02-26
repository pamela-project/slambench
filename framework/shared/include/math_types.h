/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef MATH_TYPES_H_
#define MATH_TYPES_H_


typedef struct sb_int2 {
  int x,y;
} sb_int2;


typedef struct sb_uint2 {
  unsigned int x,y;
} sb_uint2;


typedef struct sb_uint3 {
  unsigned int x,y,z;
} sb_int3;

typedef struct sb_uchar3 {
  unsigned char x,y,z;
} sb_uchar3;

typedef struct sb_uchar4 {
  unsigned char x,y,z,w;
} sb_uchar4;


typedef struct sb_float3 {
  float x,y,z;
} sb_float3;


typedef struct sb_float4 {
  float x,y,z,w;
} sb_float4;



inline sb_int2 make_sb_int2( int a, int b) {return {a,b};}

inline sb_uint2 make_sb_uint2(unsigned int a,unsigned int b) {return {a,b};}

inline sb_uchar3 make_sb_uchar3(unsigned char a,unsigned char b, unsigned char c) {return {a,b,c};}

inline sb_uchar4 make_sb_uchar4(unsigned char a,unsigned char b, unsigned char c, unsigned char d) {return {a,b,c,d};}

inline sb_uint3  make_sb_uint3(unsigned int a,unsigned int b, unsigned int c) {return {a,b,c};}

inline sb_float3 make_sb_float3(float a,float b, float c) {return {a,b,c};}

inline sb_float4 make_sb_float4(float a,float b, float c, float d) {return {a,b,c,d};}


#endif /* MATH_TYPES_H_ */
