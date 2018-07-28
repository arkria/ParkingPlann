import KDtreeKnn.kdtree as kdtree, KDtreeKnn.Distance as ds
from math import  sin, cos


B = 1.0 #[m] distance from rear to vehicle back end
C = 3.7 #[m] distance from rear to vehicle front end
I = 2.0 #[m] width of vehicle
WBUBBLE_DIST = (B+C)/2.0-B #[m] distance from rear and the center of whole bubble
WBUBBLE_R = (B+C)/2.0 #[m] whole bubble radius

vrx = [C, C, -B, -B, C ]
vry = [-I/2.0, I/2.0, I/2.0, -I/2.0, -I/2.0]

def check_collision(x, y, yaw, kdtreeroot, ox, oy):

    for (ix, iy, iyaw) in zip(x, y, yaw)
        cx = ix + WBUBBLE_DIST*cos(iyaw)
        cy = iy + WBUBBLE_DIST*sin(iyaw)

        # Whole bubble check
        ids = inrange(kdtree, [cx, cy], WBUBBLE_R, true)
        if length(ids) == 0 continue end

        if !rect_check(ix, iy, iyaw, ox[ids], oy[ids])
            return false #collision
        end
    end

    return true #OK

end

