#ifndef _CONSTANTS_H
#define _CONSTANTS_H

//@TODO verify
#define UNIT_FIX 150 

/* Width and height of occupancy grids */
#define OG_WIDTH (2 * UNIT_FIX)
#define OG_HEIGHT (2 * UNIT_FIX)

/* Width and height of occupancy grid quadrants (1/4) */
#define OG_SEC_W (OG_WIDTH  / 2)
#define OG_SEC_H (OG_HEIGHT / 2)

/* Himm configuration */
#define HIMM_THRESHOLD_MAX 30
#define HIMM_THRESHOLD_MIN 0

#endif _CONSTANTS_H
