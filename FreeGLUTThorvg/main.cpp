/*
 * This is an example of using FreeGLUT with Thorvg
 * The example will load svg image and display it to the window.
 * FreeGLUTThorvg, Created by Michal Maciola (m.maciola@samsung.com) on 17/11/2021.
 */

#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <thorvg.h>
#include <thread>

using namespace std;
using namespace tvg;

#define WIDTH 400
#define HEIGHT 400
#define FPS 60
#define INTERVAL (1000/FPS)

GLubyte* PixelBuffer = new GLubyte[WIDTH * HEIGHT * 4];

static CanvasEngine tvgEngine = CanvasEngine::Sw;
static unique_ptr<SwCanvas> swCanvas;
static bool needInvalidation = false;
static int mousePosition[] = {0, 0};
static unique_ptr<Shape> Rect = NULL;

void createThorvgView() {
    // Initialize the engine
    auto threads = thread::hardware_concurrency();
    if (threads > 0) --threads;
    
    if (Initializer::init(tvgEngine, threads) != Result::Success) {
        printf("Thorvg init failed: Engine is not supported\n");
        return;
    }
    
    // Create canvas using ABGR8888. Note: ABGR8888 is only partly supported, default is ARGB8888
    swCanvas = SwCanvas::gen();
    swCanvas->target((uint32_t *) PixelBuffer, WIDTH, WIDTH, HEIGHT, SwCanvas::ABGR8888);
    
    // Push white background
    auto shape = Shape::gen();
    shape->appendRect(0, 0, WIDTH, HEIGHT, 0, 0);
    shape->fill(255, 255, 255, 255);
    swCanvas->push(move(shape));
    
    // Push picture
    auto picture = Picture::gen();
    if (picture->load("/Users/mtm/Development/samsung/thorvg-mm/src/examples/images/tiger.svg") != Result::Success) return;
    picture->size(WIDTH, HEIGHT);
    swCanvas->push(move(picture));
    
    // Create rect
    Rect = Shape::gen();
    swCanvas->push(unique_ptr<Shape>(Rect.get()));
    
    // Draw, will be synced later
    swCanvas->draw();
}

// Draw the buffer
void display() {
    // Sync thorvg drawing
    swCanvas->sync();
    
    // Clear the buffer (not needed as buffer is full window sized)
    glClear(GL_COLOR_BUFFER_BIT);
    
    // Flip vertically, as (0,0) is in the topleft corner
    glRasterPos2f(-1,1);
    glPixelZoom(1, -1);
    
    // Draw pixels
    glDrawPixels(WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, PixelBuffer);
    
    // Flush
    glFlush();
}

// Handle mouse events
void mouse(int button, int state, int x, int y) {
    // Handle only left mouse button
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            // Store begin mouse position
            mousePosition[0] = x;
            mousePosition[1] = y;
            
            // Reset rect shape
            Rect->reset();
            swCanvas->update(Rect.get());
            
            // Draw, will be synced later
            swCanvas->draw();
        }
    }
}
void mouseMotion(int mx, int my) {
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
    
    // Create rect shape
    Rect->reset();
    Rect->appendRect(coordinates[0], coordinates[1], coordinates[2], coordinates[3], 0, 0);
    Rect->fill(0x00, 0xba, 0xcc, 0xa0);
    swCanvas->update(Rect.get());
    
    // Draw, will be synced later
    swCanvas->draw();
    
    // Call for invalidation
    needInvalidation = true;
}

// Handle the timer event
void timer(int value) {
    // Invalidate if needed
    if (needInvalidation)
        glutPostRedisplay();
    
    // Call the function timer() after INTERVAL time
    glutTimerFunc(INTERVAL, timer, value);
}

// Handle close event
void close() {
    swCanvas->clear(false);
    Initializer::term(tvgEngine);
}

// Initializes GLUT, the display mode, and main window
int main(int argc, char** argv) {
    // Use a single buffered window in RGB mode
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

    // Position, size and entitle window
    glutInitWindowSize(WIDTH, HEIGHT);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("FreeGLUT with Thorvg");
    
    // Create thorvg view
    createThorvgView();

    // Call the function display() on GLUT window repaint
    glutDisplayFunc(display);
    
    // Call the function mouse() on mouse button pressed and mouseMotion() on mouse move with pressed button
    glutMouseFunc(mouse);
    glutMotionFunc(mouseMotion);
    
    // Call the function timer() after INTERVAL time
    glutTimerFunc(INTERVAL, timer, 0);
    
    // Set closing function
    glutWMCloseFunc(close);

    // Start main loop
    glutMainLoop();
}
