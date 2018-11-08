#ifndef _CONSTANTS_H
#define _CONSTANTS_H

//type be used within cells of occupancy grids
typedef double OGCellType;

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

/* Hokuyo */
#define HOKUYO_ANGLE_MIN -2.356194
#define HOKUYO_ANGLE_MAX  2.092350
#define HOKUYO_ANGLE_INC  0.006136
#define HOKUYO_RANGE_MIN  0.02
#define HOKUYO_RANGE_MAX  5.60

#endif /* _CONSTANTS_H */
