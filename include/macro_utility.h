
#pragma once

#define OFF 0
#define ON  1

#define UNKNOWN_CELL_LABEL 2.0
#define NOT_TRAV_CELL_LABEL  0.0
#define TRAV_CELL_LABEL  1.0

#define LOWEST_Z_VALUE -100.f
#define HIGHEST_Z_VALUE 1000.0f

// RGB data for distinguish road from other classes of objects
// values manually set from https://github.com/PRBonn/semantic-kitti-api/blob/master/config/semantic-kitti.yaml
#define IS_ROAD(r, g, b)            ( (r) == 255 && (g) == 0   && (b) == 255 )
#define IS_SIDEWALK(r, g, b)        ( (r) == 75  && (g) == 0   && (b) == 75  )
#define IS_PARKING(r, g, b)         ( (r) == 255 && (g) == 150 && (b) == 255 )
#define IS_OTHER_GROUND(r, g, b)    ( (r) == 70  && (g) == 0   && (b) == 175 )
#define IS_LANE_MARKING(r, g, b)    ( (r) == 150 && (g) == 255 && (b) == 170 )

#define POINTBELONGSTOROAD(r, g, b) ( ( \
                    IS_ROAD           ( (r), (g), (b) )  \
                    || IS_SIDEWALK    ( (r), (g), (b) )  \
                    || IS_PARKING     ( (r), (g), (b) )  \
                    || IS_OTHER_GROUND( (r), (g), (b) )  \
                    || IS_LANE_MARKING( (r), (g), (b) )  \
                    ) )

#define RATE(num, den) ( ( (den) != 0) ? ( (float) (num) / (float) (den) ) : .0f )

#define TO_PERC(num) ( (num) * 100.0f )

#define ISUNKNOWN(label)  ( (label) == UNKNOWN_CELL_LABEL  )
#define ISNOTTRAVERSABLE(label)  ( (label) == NOT_TRAV_CELL_LABEL )
#define ISTRAVERSABLE(label)     ( (label) == TRAV_CELL_LABEL     )

#define NUM2SQ(x) ((x)*(x))

#define SCALAR_PRODUCT(vec1, vec2) ( ((vec1).x*(vec2).x + (vec1).y*(vec2).y + (vec1).z*(vec2).z) )
#define SCALAR_PRODUCT_2p(vec1, vec2) ( ((vec1).x*(vec2)->x + (vec1).y*(vec2)->y + (vec1).z*(vec2)->z) )

#define MAGNITUDE(vec) ( sqrtf(NUM2SQ((vec).x) + NUM2SQ((vec).y) + NUM2SQ((vec).z)) )