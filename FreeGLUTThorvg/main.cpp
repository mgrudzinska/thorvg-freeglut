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
//#define BENCHMARK

#ifdef BENCHMARK
#include <chrono>
#include "string.h"
#endif

using namespace std;
using namespace tvg;

#define WIDTH 800
#define HEIGHT 800
#define FPS 60
#define INTERVAL (1000/FPS)

GLubyte* PixelBuffer = new GLubyte[WIDTH * HEIGHT * 4];

static CanvasEngine tvgEngine = CanvasEngine::Sw;
static unique_ptr<SwCanvas> swCanvas;
static bool needInvalidation = false;
static int mousePosition[] = {0, 0};
static Shape* _pShape = nullptr;
static Shape* _pPath = nullptr;
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

    HandWriting()
    {
        cmdsReserved = 2;
        cmds = (PathCommand*)malloc(cmdsReserved * sizeof(PathCommand));
        ptsReserved = 2;
        pts = (Point*)malloc(ptsReserved * sizeof(Point));
    }

    ~HandWriting()
    {
        reset();
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
        if (cmds) free(cmds);
        cmds = nullptr;
        cmdsSize = cmdsReserved = 0;

        if (pts) free(pts);
        pts = nullptr;
        ptsSize = ptsReserved = 0;
    }
};
static HandWriting* hw = nullptr;


void createThorvgView(uint32_t threads) {
    // Initialize the engine
    if (Initializer::init(tvgEngine, 0) != Result::Success) {
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
    cout << "Time difference = " << chrono::duration_cast<chrono::microseconds>(end - begin).count() << "[µs], Memory used = " << memory << endl;
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
        if (!hw) hw = new HandWriting();
        hw->addPathCommand(PathCommand::MoveTo);
        hw->addPoint(x, y);
    } else {
        _pPath = nullptr;
        hw->reset();
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
    _pPath->stroke(2);
    swCanvas->update(_pPath);
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
    delete hw;
}


// Initializes GLUT, the display mode, and main window
int main(int argc, char** argv) {
    // Use a single buffered window in RGB mode
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

    // Position, size and entitle window
    glutInitWindowSize(WIDTH, HEIGHT);
    glutInitWindowPosition(700, 100);
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
