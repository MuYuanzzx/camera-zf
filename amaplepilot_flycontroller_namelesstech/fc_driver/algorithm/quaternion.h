#ifndef __QUATERNION_H
#define __QUATERNION_H


void quaternion_to_cb2n(float *q,float *cb2n);
void euler_to_quaternion(float *rpy,float *q);
void quaternion_add(float *q_srv,float *delta,float *q_dst);
void quaternion_sub(float *q_srv,float *delta,float *q_dst);
void quaternion_mul(float *q1_src, float *q2_src,float *q_dst);
void quaternion_conjugate(float *q_srv,float *q_dst);
void quaternion_scale(float *q_srv,float scale,float *q_dst);
void quaternion_normalize(float *q);
void quaternion_from_cb2n(float *q,float *cb2n);
void quaternion_to_euler(float *q,float *rpy);

void quaternion_rungekutta4(float *q, float *w, float dt, int normalize);


void vector_from_bodyframe2earthframe(vector3f *bf,vector3f *ef,float *cb2n);
void vector_from_earthframe2bodyframe(vector3f *ef,vector3f *bf,float *cb2n);

#endif


