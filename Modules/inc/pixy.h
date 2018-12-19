#ifndef __PIXY_H
#define __PIXY_H

#include "main.h"

typedef struct Pixy_color//单色块位置大小信息
{
	u16 Pixy_Color_Sig;//1-7 for normal signatures
	u16 Pixy_Color_PosX;  //0 to 319
	u16 Pixy_Color_PosY;  //0 to 319
	u16 Pixy_Color_Width; //1 to 320
	u16 Pixy_Color_Height;//1 to 320
}Pixy_Color;

extern Pixy_Color Pixy_Color_Inf;
extern float p_velocity_x,p_velocity_y;
extern float pixy_v_est_x,pixy_v_est_y;	
extern u16 pixy_PosX,pixy_PosY;
extern u8 pixy_available;

void pixy_Update(void);
void pixy_Velocity_estimateEKF(float ax,float ay,float dt);
void velociyt_est_reset(void);
#endif 
