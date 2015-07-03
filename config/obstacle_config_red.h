#ifndef OBSTACLE_CONFIG_H_
#define OBSTACLE_CONFIG_H_

#define NUM_OBSTACLE 7

namespace obstacle
{

double rpy[][3] = {{3.5120109, 3.74856442, 4.76836176},
		   { 0.04793009, 5.95623159, 3.47521475},
		   { 1.83101575, 3.86618623, 6.12389841},
		   { 0.89809337, 1.32163439, 6.2139846 },
		   { 0.29740188, 4.33614838, 3.2089832 },
		   { 3.48865966, 5.59539665, 1.76283863}};

double size[][3] = {{ 0.129272094328, 0.102149869881, 0.247115497245},
		    { 0.155404177609, 0.154207188695, 0.282102644495},
		    { 0.489492094983, 0.323785114969, 0.37558560033},
		    { 0.227381444253, 0.17669342002, 0.498539229839},
		    { 0.403384909498, 0.318146021643, 0.370300764032},
		    { 0.442150164327, 0.410673396992, 0.177139522113}};

double pos[][3] = {{ 1.69156545009, 1.50065518997, 0.783740435645},
		   { -1.68087943804, 0.209303548394, 0.244540540007},
		   { 1.36481346833, -0.118525861673, 0.40642873977},
		   { -1.90228256923, -1.95493361508, 0.945037848727},
		   { 0.115545997987, 1.91270915458, 1.86724796016},
		   { 1.25234286743, -0.218774308385, 0.797823685763}};
}

#endif