/*
 * This is an example of using FreeGLUT with Thorvg
 * The example will load svg image and display it to the window.
 * FreeGLUTThorvg, Created by Michal Maciola (m.maciola@samsung.com) on 17/11/2021.
 */

#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif
#include <thorvg.h>
#include <thread>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

// For benchmark
// #define BENCHMARK

#ifdef BENCHMARK
#include <chrono>
#include "string.h"
#endif


#define SHAPE_RECOG
//#define APPROX
#define EXACT

#if defined(APPROX) || defined(SHAPE_RECOG)
  #include "DP_alg.hpp"
  #ifdef APPROX
    #include <cstring>
    #define APPROX_BUF_SIZE 30
    #define APPROX_EPSILON 2
  #endif
  #ifdef SHAPE_RECOG
    #define SHAPE_RECOG_EPSILON 0.03 //for 0.02 square often has 5 corners
    #include <float.h>
  #endif
#endif

using namespace std;
using namespace tvg;

#define WIDTH 800
#define HEIGHT 800
#define FPS 60
#define INTERVAL (1000/FPS)

#define DEF_SIZE 10

GLubyte* PixelBuffer = new GLubyte[WIDTH * HEIGHT * 4];

static CanvasEngine tvgEngine = CanvasEngine::Sw;
static unique_ptr<SwCanvas> swCanvas;
static bool needInvalidation = false;
static int mousePosition[] = {0, 0};
static Shape* _pShape = nullptr;
#ifdef EXACT
static Shape* _pPath = nullptr;
#endif
#ifdef APPROX
static Shape* _pApproxPath = nullptr;
static int _newPoints = 0;
#endif
//static bool _menuUsed = false;
static int _writingChosen = true;
static int _drawingChosen = false;


struct Color
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
};
static Color _color = {0, 0, 0, 255};

struct HandWriting
{
    PathCommand* cmds = nullptr;
    uint32_t cmdsSize = 0;
    uint32_t cmdsReserved = 0;

    Point* pts = nullptr;
    uint32_t ptsSize = 0;
    uint32_t ptsReserved = 0;

#ifdef APPROX
    Point* approxBuf = nullptr;
#endif

    HandWriting()
    {
        cmdsReserved = DEF_SIZE;
        cmds = (PathCommand*)malloc(cmdsReserved * sizeof(PathCommand));
        ptsReserved = DEF_SIZE;
        pts = (Point*)malloc(ptsReserved * sizeof(Point));
#ifdef APPROX
        approxBuf = (Point*)malloc(APPROX_BUF_SIZE * sizeof(Point));
#endif
    }

    ~HandWriting()
    {
        if (cmds) free(cmds);
        cmds = nullptr;
        cmdsSize = cmdsReserved = 0;

        if (pts) free(pts);
        pts = nullptr;
        ptsSize = ptsReserved = 0;

#ifdef APPROX
        if (approxBuf) free(approxBuf);
        approxBuf = nullptr;
#endif
    }

    bool addPathCommand(PathCommand cmd) {
        if (cmdsSize + 1 > cmdsReserved) {
            cmdsReserved = (cmdsSize + 1) * 2;
            cmds = (PathCommand*)realloc(cmds, cmdsReserved * sizeof(PathCommand));
            if (!cmds) return false;
        }

        cmds[cmdsSize++] = cmd;

        return true;
    }

    bool addPoint(int x, int y) {
        if (ptsSize + 1 > ptsReserved) {
            ptsReserved = (ptsSize + 1) * 2;
            pts = (Point*)realloc(pts, ptsReserved * sizeof(Point));
            if (!pts) return false;
        }
        pts[ptsSize].x = (float)x;
        pts[ptsSize].y = (float)y;
        ptsSize++;
        return true;
    }

    void reset() {
        cmdsSize = 0;
        cmdsReserved = DEF_SIZE;
        cmds = (PathCommand*)realloc(cmds, cmdsReserved * sizeof(PathCommand));;

        ptsSize = 0;
        ptsReserved = DEF_SIZE;
        pts = (Point*)realloc(pts, ptsReserved * sizeof(Point));
    }
};
#ifdef EXACT
static HandWriting* hw = nullptr;
#endif
#ifdef APPROX
static HandWriting* ahw = nullptr;
#endif


#ifdef SHAPE_RECOG

#define CLOSING_TOLERANCE_POINTS 10
#define CLOSING_TOLERANCE_RADIUS 0.1
#define RAD_TO_DEG 57.29578f   // 180/M_PI
#define DEG_TO_RAD 0.017453f   // M_PI/180
#define PI_2 1.570796f         // M_PI/2
#define TWO_PI 6.283185f       // 2*M_PI
#define PI_6 0.523599f         // M_PI/6.0f
#define ORTHOGONALITY_TOLERANCE   10 * DEG_TO_RAD  // TODO: choose smaller value - 10 is only for tests
#define ANGLES_EQUALITY_TOLERANCE 10 * DEG_TO_RAD  // TODO: choose smaller value - 10 is only for tests
#define SLOPE_TOLERANCE           10 * DEG_TO_RAD  // TODO: choose smaller value - 10 is only for tests


inline float exactLength(const Point& p1, const Point& p2)
{
    Point distance = {p2.x - p1.x, p2.y - p1.y};

    return sqrtf(distance.x * distance.x + distance.y * distance.y);
}


// aplha max plus beta min (alpha = 1, beta = 3/8 -> largest error 6.8%)
inline float approxLength(const Point& p1, const Point& p2)
{
    Point distance = {p2.x - p1.x, p2.y - p1.y};
    if (distance.x < 0) distance.x = -distance.x;
    if (distance.y < 0) distance.y = -distance.y;

    return (distance.x > distance.y) ? (distance.x + distance.y * 0.375f) : (distance.y + distance.x * 0.375f);
}


float approxPerimiter(const Point* contour, int contourSize)
{
    float peri = 0.0f;

    for (int i = 1; i < contourSize; ++i)
        peri += approxLength(contour[i - 1], contour[i]);

    return peri;
}


Point findCenter(const Point* contour, int contourSize)
{
    float cx = 0.0f, cy = 0.0f;

    for (int i = 0; i < contourSize; ++i) {
        cx += contour[i].x;
        cy += contour[i].y;
    }
    cx /= contourSize;
    cy /= contourSize;

    return {cx, cy};
}


// check whether the first and the last CLOSING_TOLERANCE_POINTS points lie in the same area of a CLOSING_TOLERANCE_RADIUS radius (% of the perimiter)
// TODO: should the tolerance depend on the stroke width ?
bool closedContour(const Point* contour, int contourSize, float perimiter)
{
    int tolerance = (2 * CLOSING_TOLERANCE_POINTS > contourSize ? contourSize / 2 : CLOSING_TOLERANCE_POINTS);
    Point centerBegin = findCenter(contour, tolerance);
    Point centerEnd = findCenter(contour + (contourSize - tolerance), tolerance);
    Point center = {(centerBegin.x + centerEnd.x) / 2.0f, (centerBegin.y + centerEnd.y) / 2.0f};
    float radius = perimiter * CLOSING_TOLERANCE_RADIUS;

    for (int i = 0; i < tolerance; ++i) {
        if (approxLength(contour[i], center) > radius ||
            approxLength(contour[contourSize - 1 - i], center) > radius) {
                return false;
        }
    }
    return true;
}


float angle(const Point& A, const Point& B, const Point& C) {
    float lenAB = exactLength(A, B);
    Point normalizedAB = {(A.x - B.x) / lenAB, (A.y - B.y) / lenAB};
    float lenBC = exactLength(B, C);
    Point normalizedBC = {(C.x - B.x) / lenBC, (C.y - B.y) / lenBC};

    return acos(normalizedAB.x * normalizedBC.x + normalizedAB.y * normalizedBC.y);
}


// TODO: some parts of the code can be unified
Shape* recognizeTriangle(Point* triangle)
{
    const int vertices = 3;
    Point center = findCenter(triangle, vertices);

    bool isOrthogonal = false;
    bool isEquilateral = true;
    bool isIsosceles = false;

    uint8_t orthogonalVertex;
    uint8_t topVertex = 0;
    uint8_t isoscelesVertex;
    uint8_t maxSideVertex;

    float maxSideLength = 0;
    float r = 0.0f;

    float slope;

    for (int i = 0; i < vertices; ++i) {
        auto length = exactLength(triangle[i], triangle[(i + 1) % vertices]);
        if (maxSideLength < length) {
            maxSideLength = length;
            maxSideVertex = i;
            if (fabsf(triangle[(i + 1) % vertices].x - triangle[i].x) < FLT_EPSILON) {
                slope = PI_2;
                // TODO: check this case - is the y sign important?
            } else {
                slope = atan((triangle[(i + 1) % vertices].y - triangle[i].y) / (triangle[(i + 1) % vertices].x - triangle[i].x));
            }
        }
        if (triangle[i].y < triangle[topVertex].y) topVertex = i;
        r += exactLength(triangle[i], center);
    }
    r /= vertices;

    float angles[vertices];

    for (int i = 0; i < vertices; ++i) {
        Point A = triangle[i];
        Point B = triangle[(i + 1) % vertices];
        Point C = triangle[(i + 2) % vertices];

        angles[(i + 1) % vertices] = angle(A, B, C);
    }

    for (int i = 0; i < vertices; ++i) {
        //isosceles triangle
        if (fabsf(angles[i] - angles[(i + 1) % vertices]) < ANGLES_EQUALITY_TOLERANCE) {
            isoscelesVertex = i;
            isIsosceles = true;
        } 
        //equilateral triangle
        if (fabsf(angles[i] - M_PI / 3.0f) > ANGLES_EQUALITY_TOLERANCE) {
            isEquilateral = false;
        }
        //orthogonal triangle
        if (fabsf(angles[i] - PI_2) < ORTHOGONALITY_TOLERANCE) {
            isOrthogonal = true;
            orthogonalVertex = i;
        }
    }
    //either isosceles with two angles ~90deg or orthogonal
    if (isIsosceles && (orthogonalVertex == isoscelesVertex || orthogonalVertex == ((isoscelesVertex + 1) % vertices)))
        isOrthogonal = false;

    auto ABC = Shape::gen();

    if (isEquilateral) {
        float cosSlope = cos(slope);
        float sinSlope = sin(slope);
        Point transformed = {cosSlope * (triangle[(maxSideVertex + 1) % vertices].x - triangle[maxSideVertex].x) + sinSlope * (triangle[(maxSideVertex + 1) % vertices].y - triangle[maxSideVertex].y) + triangle[maxSideVertex].x,
                            -sinSlope * (triangle[(maxSideVertex + 1) % vertices].x - triangle[maxSideVertex].x) + cosSlope * (triangle[(maxSideVertex + 1) % vertices].y - triangle[maxSideVertex].y) + triangle[maxSideVertex].y};

        int signAB = 1;
        if (triangle[(maxSideVertex + 1) % vertices].x > triangle[maxSideVertex].x) signAB = -1;

        float dAngle = TWO_PI / vertices;
        ABC->moveTo(r * sin(dAngle * (-0.5f)), r * cos(dAngle * (-0.5f)));
        for (int i = 1; i < vertices; ++i) {
            ABC->lineTo(r * sin(dAngle * (i - 0.5f)), r * cos(dAngle * (i - 0.5f)));
        }
        ABC->close();

        float rotAngle = angle({2 * center.x, center.y}, center, triangle[topVertex]) + PI_6;
        int n = round(rotAngle / PI_6);
        if (fabsf(n * PI_6 - rotAngle) < ANGLES_EQUALITY_TOLERANCE) {
            rotAngle = n * PI_6;
        }

        ABC->rotate(-rotAngle * RAD_TO_DEG);
        ABC->translate(center.x, center.y);
        return ABC.release();
    } else if (isOrthogonal) {
        Point A = triangle[maxSideVertex];

        if (fabsf(slope) < SLOPE_TOLERANCE) slope = 0.0f;
        else if (fabsf(slope - PI_2) < SLOPE_TOLERANCE) slope = PI_2;
        else if (fabsf(slope + PI_2) < SLOPE_TOLERANCE) slope = -PI_2;
        float cosSlope = cos(slope);
        float sinSlope = sin(slope);

        Point transformedB = {cosSlope * (triangle[(maxSideVertex + 1) % vertices].x - A.x) + sinSlope * (triangle[(maxSideVertex + 1) % vertices].y - A.y) + A.x,
                             -sinSlope * (triangle[(maxSideVertex + 1) % vertices].x - A.x) + cosSlope * (triangle[(maxSideVertex + 1) % vertices].y - A.y) + A.y};

        int signAB = 1;
        if (transformedB.x < A.x) signAB = -1;

        Point B = {A.x + signAB * maxSideLength, A.y};

        float dAngle = (angles[(maxSideVertex + 2) % vertices] - PI_2) / 2.0f;
        float angle1 = angles[maxSideVertex] + dAngle;
        float angle2 = angles[(maxSideVertex + 1) % vertices] + dAngle;
        if (isIsosceles) angle1 = angle2 = M_PI / 4.0f;

        float side = sin(angle1) * maxSideLength;
        float dx = cos(angle2) * side;
        float dy = -sin(angle2) * side;

        int signBC = 1;
        float transformedCy = -sinSlope * (triangle[(maxSideVertex + 2) % vertices].x - A.x) + cosSlope * (triangle[(maxSideVertex + 2) % vertices].y - A.y) + A.y;
        if (transformedCy > transformedB.y) signBC = -1;
        Point C = {B.x - dx * signAB, B.y + dy * signBC};

        Point transC = {cos(-slope) * (C.x - A.x) + sin(-slope) * (C.y - A.y) + A.x,
                       -sin(-slope) * (C.x - A.x) + cos(-slope) * (C.y - A.y) + A.y};

        if (fabsf(transC.x + A.x) > FLT_EPSILON) {
            float align = atan((transC.y - A.y) / (transC.x - A.x));
            float check[5] = {0.0f, M_PI, -M_PI, -PI_2, PI_2};
            for (int i = 0; i < 5; ++i) {
                if (fabsf(align - check[i]) < SLOPE_TOLERANCE) {
                    slope -= (align - check[i]);
                    break;
                }
            }
        }

        ABC->moveTo(0, 0);
        ABC->lineTo(B.x - A.x, B.y - A.y);
        ABC->lineTo(C.x - A.x, C.y - A.y);
        ABC->close();
        ABC->rotate(slope * RAD_TO_DEG);
        ABC->translate(A.x, A.y);

        return ABC.release();
    } else if (isIsosceles) {
        Point A = triangle[isoscelesVertex];

        if (isoscelesVertex != maxSideVertex) {
            if (fabsf(triangle[(isoscelesVertex + 1) % vertices].x - triangle[isoscelesVertex].x) < FLT_EPSILON) {
                slope = PI_2;
            } else {
                slope = atan((triangle[(isoscelesVertex + 1) % vertices].y - triangle[isoscelesVertex].y) / (triangle[(isoscelesVertex + 1) % vertices].x - triangle[isoscelesVertex].x));
            }
        }

        if (fabsf(slope) < SLOPE_TOLERANCE) slope = 0.0f;
        else if (fabsf(slope - PI_2) < SLOPE_TOLERANCE) slope = PI_2;
        else if (fabsf(slope + PI_2) < SLOPE_TOLERANCE) slope = -PI_2;
        float cosSlope = cos(slope);
        float sinSlope = sin(slope);

        Point transformedB = {cosSlope * (triangle[(isoscelesVertex + 1) % vertices].x - A.x) + sinSlope * (triangle[(isoscelesVertex + 1) % vertices].y - A.y) + A.x,
                             -sinSlope * (triangle[(isoscelesVertex + 1) % vertices].x - A.x) + cosSlope * (triangle[(isoscelesVertex + 1) % vertices].y - A.y) + A.y};

        int signAB = 1;
        if (transformedB.x < A.x) signAB = -1;

        Point B = {A.x + signAB * exactLength(A, triangle[(isoscelesVertex + 1) % vertices]), A.y};

        float angle = (angles[isoscelesVertex] + angles[(isoscelesVertex + 1) % vertices]) / 2.0f;

        float side = exactLength(A, triangle[(isoscelesVertex + 1) % vertices]) / 2.0f / cos(angle);
        float dx = exactLength(A, triangle[(isoscelesVertex + 1) % vertices]) / 2.0f;
        float dy = -sin(angle) * side;

        int signBC = 1;
        float transformedCy = -sinSlope * (triangle[(isoscelesVertex + 2) % vertices].x - A.x) + cosSlope * (triangle[(isoscelesVertex + 2) % vertices].y - A.y) + A.y;
        if (transformedCy > transformedB.y) signBC = -1;
        Point C = {B.x - dx * signAB, B.y + dy * signBC};

        Point transC = {cos(-slope) * (C.x - A.x) + sin(-slope) * (C.y - A.y) + A.x,
                       -sin(-slope) * (C.x - A.x) + cos(-slope) * (C.y - A.y) + A.y};

        ABC->moveTo(0, 0);
        ABC->lineTo(B.x - A.x, B.y - A.y);
        ABC->lineTo(C.x - A.x, C.y - A.y);
        ABC->close();
        ABC->rotate(slope * RAD_TO_DEG);
        ABC->translate(A.x, A.y);

        return ABC.release();
    }
 
    ABC->moveTo(triangle[0].x, triangle[0].y);
    ABC->lineTo(triangle[1].x, triangle[1].y);
    ABC->lineTo(triangle[2].x, triangle[2].y);
    ABC->close();

    return ABC.release();
}


Shape* recognizeRectangle(const Point* rectangle)
{
    const int vertices = 4;

    // final result parameters
    float x, y, w, h;
    float alpha;
    Point shift;

    float area = FLT_MAX;

    for (int i = 0; i < vertices; ++i) {
        Point start = rectangle[i];
        Point end = rectangle[(i + 1) % vertices];

        float slope = atan((end.y - start.y) / (end.x - start.x));
        float cosSlope = cos(slope);
        float sinSlope = sin(slope);

        Point min = {0, 0};
        Point max = {0, 0};

        // points are translated so that rect[0] = (0,0) and rotated so the start-end line is axis aligned
        for (int j = 1 + i; j < vertices + i; ++j) {
            Point transformed = {cosSlope * (rectangle[j % vertices].x - start.x) + sinSlope * (rectangle[j % vertices].y - start.y),
                                -sinSlope * (rectangle[j % vertices].x - start.x) + cosSlope * (rectangle[j % vertices].y - start.y)};
            if (transformed.x < min.x) min.x = transformed.x;
            if (transformed.y < min.y) min.y = transformed.y;
            if (transformed.x > max.x) max.x = transformed.x;
            if (transformed.y > max.y) max.y = transformed.y;
        }

        float width = max.x - min.x;
        float height = max.y - min.y;
        if (width * height < area) {
            x = min.x;
            y = min.y; 
            w = width;
            h = height;
            shift.x = start.x;
            shift.y = start.y;
            alpha = slope;
            area = w * h;
        }
    }

    auto rect = Shape::gen();
    rect->appendRect(x, y, w, h, 0, 0);
    rect->rotate(alpha * 180.0f / M_PI);
    rect->translate(shift.x, shift.y);

    return rect.release();
}


Shape* recognizePolygon(Point* contour, int vertices)
{
    Point center = findCenter(contour, vertices);

    float r = 0.0f;
    for (int i = 0; i < vertices; ++i) {
        r += approxLength(contour[i], center);
    }
    r /= vertices;

    //TODO: juz moge wypisac punkty - zakladam zawsze ze podstawa || OX
    auto polygon = Shape::gen();

    auto angle = 2.0f * M_PI / vertices;
    polygon->moveTo(center.x + r * sin(-M_PI / vertices), center.y + r * cos(-M_PI / vertices));
    for (int i = 1; i < vertices; ++i) {
        polygon->lineTo(center.x + r * sin(angle * (i - 0.5)), center.y  + r * cos(angle * (i - 0.5)));
    }
    polygon->close();

    return polygon.release();
}


Shape* recognizeCircle(Point* contour, int vertices)
{
    Point center = findCenter(contour, vertices);
    uint8_t rMaxVertex;

    float rMin = FLT_MAX, rMax = 0.0f;
    for (int i = 0; i < vertices; ++i) {
        auto length = approxLength(contour[i], center);
        if (rMin > length) rMin = length;
        if (rMax < length) {
            rMax = length;
            rMaxVertex = i;
        }
    } 

    auto circle = Shape::gen();
    circle->appendCircle(0, 0, rMax, rMin);

    float slope = angle({2 * center.x, center.y}, center, contour[rMaxVertex]);
    int sign = (contour[rMaxVertex].y < center.y ? -1 : 1);
    circle->rotate(sign * slope * RAD_TO_DEG);
    circle->translate(center.x, center.y);

    return circle.release();
}


Shape* recognizeShape(const Point* contour, int contourSize)
{
    if (contourSize < 2) return nullptr;

    float peri = approxPerimiter(contour, contourSize);
    if (!closedContour(contour, contourSize, peri)) return nullptr;

    auto shapeRecogBuf = (Point*)malloc(contourSize * sizeof(Point));
    int vertices = approxPolyDP(contour, contourSize, shapeRecogBuf, SHAPE_RECOG_EPSILON * peri, true);

    Shape* shape = nullptr;

    //TODO: compare the areas of the original and the recognized shapes? or the outermost points? some check has to be done
    if (vertices == 2) {
    } else if (vertices == 3) {
        shape = recognizeTriangle(shapeRecogBuf);
    } else if (vertices == 4) {
        shape = recognizeRectangle(shapeRecogBuf);
//    } else if (vertices < 7) {
//        shape = recognizePolygon(shapeRecogBuf, vertices);
    } else {
        shape = recognizeCircle(shapeRecogBuf, vertices);
    }

    free(shapeRecogBuf);

    return shape;
}
#endif


void createThorvgView(uint32_t threads) {
    if (Initializer::init(tvgEngine, threads) != Result::Success) {
        cerr << "Thorvg init failed: Engine is not supported" << endl;;
        return;
    }

    // Create canvas using ABGR8888. Note: ABGR8888 is only partly supported, default is ARGB8888
    swCanvas = SwCanvas::gen();
    swCanvas->target((uint32_t *) PixelBuffer, WIDTH, WIDTH, HEIGHT, SwCanvas::ABGR8888);

    // Push white background
    auto background = Shape::gen();
    background->appendRect(0, 0, WIDTH, HEIGHT, 0, 0);
    background->fill(255, 255, 255, 255);
    swCanvas->push(move(background));

    // Draw, will be synced later
    swCanvas->draw();
}

#ifdef BENCHMARK
string getVmSizeValue() {
    FILE* file = fopen("/proc/self/status", "r");
    if (!file) return string("UNKNOWN");
    char* result = nullptr;
    char line[128];

    while (fgets(line, 128, file) != NULL) {
        if (strncmp(line, "VmSize:", 7) == 0) {
            result = line + 7;
            while (*result < '0' || *result > '9') result++;
            char* p = strchr(result, 'B');
            if (p) p[1] = '\0';
            break;
        }
    }
    fclose(file);
    return string(result);
}
#endif

// Draw the buffer
void display() {
#ifdef BENCHMARK
    auto begin = chrono::steady_clock::now();
#endif

    // Sync thorvg drawing
    swCanvas->sync();

    // Clear the buffer (not needed as buffer is full window sized)
    glClear(GL_COLOR_BUFFER_BIT);

    // Flip vertically, as (0,0) is in the topleft corner
    glRasterPos2f(-1, 1);
    glPixelZoom(1, -1);

    // Draw pixels
    glDrawPixels(WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, PixelBuffer);

    // Flush
    glFlush();

#ifdef BENCHMARK
    auto end = chrono::steady_clock::now();
    auto memory = getVmSizeValue();
    cout << "Time difference = " << chrono::duration_cast<chrono::microseconds>(end - begin).count() << " [µs], Memory used = " << memory << endl;
#endif
}


void handleDrawing(int state, int x, int y) {
    if (state == GLUT_DOWN) {
        // Store begin mouse position
        mousePosition[0] = x;
        mousePosition[1] = y;
    } else {
        // Reset _pShape
        _pShape = nullptr;
    }
}


void handleWriting(int state, int x, int y) {
    if (state == GLUT_DOWN) {
#ifdef EXACT
        if (!hw) hw = new HandWriting();
        hw->addPathCommand(PathCommand::MoveTo);
        hw->addPoint(x, y);
#endif
#ifdef APPROX
        if (!ahw) ahw = new HandWriting();
        ahw->addPathCommand(PathCommand::MoveTo);
        ahw->addPoint(x, y);
        _newPoints++;
#endif
    } else {
#ifdef EXACT
  #ifdef SHAPE_RECOG
        if (auto polygon = recognizeShape(hw->pts, hw->ptsSize)) {
            polygon->stroke(4);
            polygon->stroke(0, 0, 255, 200);
            swCanvas->push(unique_ptr<Shape>(polygon));
            swCanvas->draw();
            display();
        }
  #endif
        _pPath = nullptr;
        cout << "The exact data size: pts - " << hw->ptsSize << ", cmds - " << hw->cmdsSize << endl;
        hw->reset();
#endif

#ifdef APPROX
        _pApproxPath = nullptr;
        cout << "The approx data size: pts - " << ahw->ptsSize << ", cmds - " << ahw->cmdsSize << endl;
        ahw->reset();
#endif
    }
}


// Handle mouse events
// Handle left mouse button - the right one is caught by the menu
// TODO - for now we'll enter here even if only the menu was chosen - _menuUsed is false here
void mouse(int button, int state, int x, int y) {
    if (_drawingChosen) handleDrawing(state, x, y);
    else if (_writingChosen) handleWriting(state, x, y);
}


void handleDrawing(int mx, int my) {
    // Create cordinates
    int coordinates[4];
    if (mousePosition[0] <= mx) {
        coordinates[0] = mousePosition[0];
        coordinates[2] = mx - mousePosition[0];
    } else {
        coordinates[0] = mx;
        coordinates[2] = mousePosition[0] - mx;
    }
    if (mousePosition[1] <= my) {
        coordinates[1] = mousePosition[1];
        coordinates[3] = my - mousePosition[1];
    } else {
        coordinates[1] = my;
        coordinates[3] = mousePosition[1] - my;
    }

    // Create a shape
    if (!_pShape) {
        auto shape = Shape::gen();
        _pShape = shape.get();
        swCanvas->push(move(shape));
    } else {
        _pShape->reset();
    }

    if (_drawingChosen == 1)
      _pShape->appendRect(coordinates[0], coordinates[1], coordinates[2], coordinates[3], 0, 0);
    else if (_drawingChosen == 2)
      _pShape->appendCircle(coordinates[0] + coordinates[2] / 2, coordinates[1] + coordinates[3] / 2, coordinates[2] / 2, coordinates[3] / 2);

    _pShape->fill(0, 255, 0, 128);
    swCanvas->update(_pShape);
}


void handleWriting(int mx, int my) {
    // Create hand writing
#ifdef EXACT
    hw->addPathCommand(PathCommand::LineTo);
    hw->addPoint(mx, my);

    if (!_pPath) {
        auto path = Shape::gen();
        _pPath = path.get();
        swCanvas->push(move(path));
    } else {
        _pPath->reset();
    }

    _pPath->appendPath(hw->cmds, hw->cmdsSize, hw->pts, hw->ptsSize);
    _pPath->stroke(_color.r, _color.g, _color.b, _color.a);
    _pPath->stroke(4);
    swCanvas->update(_pPath);
#endif

#ifdef APPROX
    ahw->addPathCommand(PathCommand::LineTo);
    ahw->addPoint(mx, my);
    _newPoints++;

    if (_newPoints == APPROX_BUF_SIZE) {
        _newPoints = 0;
        int approxSize = approxPolyDP(ahw->pts + (ahw->ptsSize - APPROX_BUF_SIZE), APPROX_BUF_SIZE, ahw->approxBuf, APPROX_EPSILON, false);
        memcpy(ahw->pts + (ahw->ptsSize - APPROX_BUF_SIZE), ahw->approxBuf, approxSize * sizeof(Point));
        ahw->ptsSize -= (APPROX_BUF_SIZE - approxSize);
        ahw->cmdsSize -= APPROX_BUF_SIZE;
        for (int i = 0; i < approxSize/3; ++i)
            ahw->cmds[ahw->cmdsSize++] = PathCommand::CubicTo;
        ahw->cmdsSize += (approxSize % 3);
    }

    if (!_pApproxPath) {
        auto approxPath = Shape::gen();
        _pApproxPath = approxPath.get();
        swCanvas->push(move(approxPath));
        _newPoints = 0;
    } else {
        _pApproxPath->reset();
    }

    _pApproxPath->appendPath(ahw->cmds, ahw->cmdsSize, ahw->pts, ahw->ptsSize);
    _pApproxPath->stroke(255, 0, 0, 255);
    _pApproxPath->stroke(2);
    swCanvas->update(_pApproxPath);
#endif
}


void mouseMotion(int mx, int my) {
    if (_drawingChosen) handleDrawing(mx, my);
    else if (_writingChosen) handleWriting(mx, my);

    // Draw, will be synced later
    swCanvas->draw();

    // Call for invalidation
    needInvalidation = true;
}

// Handle the timer event
void timer(int value) {
    // Invalidate if needed
    if (needInvalidation) {
        glutPostRedisplay();
        needInvalidation = false;
    }

    // Call the function timer() after INTERVAL time
    glutTimerFunc(INTERVAL, timer, value);
}


void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    if (_color.r != r || _color.g != g || _color.b != b || _color.a != a) {
    }
    _color.r = r;
    _color.g = g;
    _color.b = b;
    _color.a = a;
}

void handleColorMenu(int index) {
    _drawingChosen = 0;
    _writingChosen = 1;
    switch (index) {
        case 1:
            setColor(255, 0, 0, 255);
            break;
        case 2:
            setColor(0, 255, 0, 255);
            break;
        case 3:
            setColor(0, 0, 255, 255);
            break;
        case 4:
            setColor(0, 0, 0, 255);
            break;
    }
}


void handleShapeMenu(int index) {
    _writingChosen = 0;
    _drawingChosen = index;
}


void handleMainMenu(int index) {
    switch (index) {
        case 1:
            handleShapeMenu(index);
            break;
        case 2:
            handleColorMenu(index);
            break;
    }
}


/*
void handleMenuStatus(int status, int x, int y) {
    if (status == GLUT_MENU_IN_USE)
        _menuUsed = true;
    else
        _menuUsed = false;
}
*/


void menu() {
    int indexShapeMenu = glutCreateMenu(handleShapeMenu);
    glutAddMenuEntry("rect", 1);
    glutAddMenuEntry("circ", 2);

    int indexColorMenu = glutCreateMenu(handleColorMenu);
    glutAddMenuEntry("red", 1);
    glutAddMenuEntry("green", 2);
    glutAddMenuEntry("blue", 3);
    glutAddMenuEntry("black", 4);

    glutCreateMenu(handleMainMenu);
    glutAddSubMenu("Draw", indexShapeMenu);
    glutAddSubMenu("Write", indexColorMenu);

    glutAttachMenu(GLUT_RIGHT_BUTTON);

//    glutMenuStatusFunc(handleMenuStatus);
}


// Handle close event
void close() {
    swCanvas->clear(true);
    Initializer::term(tvgEngine);
    delete[] PixelBuffer;
#ifdef EXACT
    delete hw;
#endif
#ifdef APPROX
    delete ahw;
#endif
}


// Initializes GLUT, the display mode, and main window
int main(int argc, char** argv) {
    // Use a single buffered window in RGB mode
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

    // Position, size and entitle window
    glutInitWindowSize(WIDTH, HEIGHT);
    glutInitWindowPosition(70, 100);
    glutCreateWindow("FreeGLUT with Thorvg");

    // Create thorvg view
    auto threads = thread::hardware_concurrency();
    if (threads > 0) --threads;
    if (argc >= 2) {
        threads = atoi(argv[1]);
    }
    cout << "Using " << threads << " threads" << endl;
    createThorvgView(threads);

    // Call the function display() on GLUT window repaint
    glutDisplayFunc(display);

    // Call the function mouse() on mouse button pressed and mouseMotion() on mouse move with pressed button
    glutMouseFunc(mouse);
    glutMotionFunc(mouseMotion);

    // Call the function timer() after INTERVAL time
    glutTimerFunc(INTERVAL, timer, 0);

    // Set closing function
    glutWMCloseFunc(close);

    // Set menu details
    menu();

    // Start main loop
    glutMainLoop();
}
