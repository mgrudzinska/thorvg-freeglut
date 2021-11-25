#include  <thorvg.h>
#include <cassert>
#include <cmath>

using namespace tvg;

struct Range {
    int start, end;
    Range(int s = 0, int e = 0) : start{s}, end{e} {};
};

int approxPolyDP_(const Point* curve, int curveSize, Point* approxCurve, double eps, bool closed, Range* ranges)
{
    #define PUSH_SLICE(slice) \
        if (top >= stacksz) {\
            stack = (Range*)realloc(stack, stacksz * 3 / 2 * sizeof(Range)); \
            stack = stack; \
            stacksz = stacksz * 3 / 2; \
        } \
        stack[top++] = slice

    #define READ_PT(pt, pos) \
        pt = curve[pos]; \
        if (++pos >= size) pos = 0

    #define READ_DST_PT(pt, pos) \
        pt = approxCurve[pos]; \
        if (++pos >= size) pos = 0

    #define WRITE_PT(pt) \
        approxCurve[newSize++] = pt

    int initIters = 3;
    Range slice(0, 0), rightSlice(0, 0);
    Point startPt = {-1000000, -1000000};
    Point endPt = {0, 0};
    Point pt = {0,0};
    int i = 0, j, pos = 0, wpos, size = curveSize, newSize = 0;
    int isClosed = closed;
    bool leEps = false;
    size_t top = 0, stacksz = curveSize;
    Range* stack = ranges;

    if (size == 0) return 0;

    eps *= eps;

    if (!isClosed) {
        rightSlice.start = size;
        endPt = curve[0];
        startPt = curve[size - 1];

        if (startPt.x != endPt.x || startPt.y != endPt.y) {
            slice.start = 0;
            slice.end = size - 1;
            PUSH_SLICE(slice);
        } else {
            isClosed = 1;
            initIters = 1;
        }
    }

    if (isClosed) {
        // 1. Find approximately two farthest points of the contour
        rightSlice.start = 0;

        for (i = 0; i < initIters; i++) {
            double dist, maxDist = 0;
            pos = (pos + rightSlice.start) % size;
            READ_PT(startPt, pos);

            for (j = 1; j < size; j++) {
                double dx, dy;

                READ_PT(pt, pos);
                dx = pt.x - startPt.x;
                dy = pt.y - startPt.y;

                dist = dx * dx + dy * dy;

                if (dist > maxDist) {
                    maxDist = dist;
                    rightSlice.start = j;
                }
            }
            leEps = (maxDist <= eps);
        }

        // 2. initialize the stack
        if (!leEps) {
            rightSlice.end = slice.start = pos % size;
            slice.end = rightSlice.start = (rightSlice.start + slice.start) % size;

            PUSH_SLICE(rightSlice);
            PUSH_SLICE(slice);
        } else WRITE_PT(startPt);
    }

    // 3. run recursive process
    while (top > 0) {
        slice = stack[--top];
        endPt = curve[slice.end];
        pos = slice.start;
        READ_PT(startPt, pos);

        if (pos != slice.end) {
            double dx, dy, dist, maxDist = 0;

            dx = endPt.x - startPt.x;
            dy = endPt.y - startPt.y;

            assert( dx != 0 || dy != 0 );

            while (pos != slice.end) {
                READ_PT(pt, pos);
                dist = fabs((pt.y - startPt.y) * dx - (pt.x - startPt.x) * dy);

                if (dist > maxDist) {
                    maxDist = dist;
                    rightSlice.start = (pos+size - 1) % size;
                }
            }
            leEps = ((maxDist * maxDist) <= eps * (dx * dx + dy * dy));
        } else {
            leEps = true;
            // read starting point
            startPt = curve[slice.start];
        }

        if (leEps) WRITE_PT(startPt);
        else {
            rightSlice.end = slice.end;
            slice.end = rightSlice.start;
            PUSH_SLICE(rightSlice);
            PUSH_SLICE(slice);
        }
    }

    if (!isClosed) WRITE_PT(curve[size - 1]);

    // last stage: do final clean-up of the approximated contour -
    // remove extra points on the [almost] straight lines.
    isClosed = closed;
    size = newSize;
    pos = isClosed ? size - 1 : 0;
    READ_DST_PT(startPt, pos);
    wpos = pos;
    READ_DST_PT(pt, pos);

    for (i = !isClosed; i < size - !isClosed && newSize > 2; i++) {
        double dx, dy, dist, successive_inner_product;
        READ_DST_PT(endPt, pos);

        dx = endPt.x - startPt.x;
        dy = endPt.y - startPt.y;
        dist = fabs((pt.x - startPt.x) * dy - (pt.y - startPt.y) * dx);
        successive_inner_product = (pt.x - startPt.x) * (endPt.x - pt.x) + (pt.y - startPt.y) * (endPt.y - pt.y);

        if (dist * dist <= 0.5 * eps * (dx * dx + dy * dy) && dx != 0 && dy != 0 && successive_inner_product >= 0) {
            newSize--;
            approxCurve[wpos] = startPt = endPt;
            if (++wpos >= size) wpos = 0;
            READ_DST_PT(pt, pos);
            i++;
            continue;
        }
        approxCurve[wpos] = startPt = pt;
        if (++wpos >= size) wpos = 0;
        pt = endPt;
    }

    if (!isClosed)
        approxCurve[wpos] = pt;

    return newSize;
}


int approxPolyDP(const Point* curve, uint32_t curveSize, Point* approxCurve, double epsilon, bool closed)
{
    if (epsilon < 0.0 || curveSize == 0) return -1;

    Point* buf = (Point*)malloc(curveSize * sizeof(Point));
    Range* ranges = (Range*)malloc(curveSize * sizeof(Range));

    int approxCurveSize = 0;
    approxCurveSize = approxPolyDP_(curve, curveSize, approxCurve, epsilon, closed, ranges);

    free(buf);
    free(ranges);

    return approxCurveSize;
}

