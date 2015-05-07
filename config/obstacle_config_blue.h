#ifndef OBSTACLE_CONFIG_H
#define OBSTACLE_CONFIG_H

#define NUM_OBSTACLE 5

namespace obstacle
{

double rpy[][3] = {{ 0, 0, 1.57},
		   { 0, 0, 1.57},
		   { 4.16858072, 3.14151269, 2.59488083},
		   { 5.79539508, 4.12217189, 5.59803713},
		   { 0, 0, 1.7},
		   { 0, 0, 0.7},
		   { 0, 0, 0}};
//{ 5.79539508, 4.12217189, 5.59803713}};

/*
double size[][3] = {{ 0.344579169627, 0.355030966789, 0.328891767837},
                    { 0.276948178771, 0.209445488218, 0.24699594717},
                    { 0.40401398715, 0.323593871228, 0.20968763284},
                    { 0.273785065864, 0.367453126297, 0.193883865543},
                    { 0.25240654217, 0.312886385638, 0.297520608834},
                    { 0.45122804681, 0.256423795383, 0.191383538763},
		    { 0.1,2,0}};
*/
double pos[][3] = {{ -0.700,  0.200, 1.250},
		   {  1.960,  0.550, 1.254},
		   { -0.350, -1.123, 0.944},
		   {  0.192,  0.501, 0.940},
		   {  0.916, -0.517, 1.230},
		   {  0.768, -0.282, 1.623},
		   {  0.200, -0.700, 1.450}};
/*
double pos[][3] = {{  0.100,  0.550, 1.254},
		   { -0.560,  0.550, 1.254},
		   { -0.350, -4.123, 0.944},
		   {  1.192,  2.501, 0.940},
		   { -0.916, -0.517, 1.230},
		   {  0.768, -0.282, 1.623},
		   {  0.200, -0.700, 1.450}};
*/
}

#endif
